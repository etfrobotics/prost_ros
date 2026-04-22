#!/usr/bin/env python3
import re
import rospy
import socket
import struct
import threading
import sys
import os
import subprocess
import base64
import xml.etree.ElementTree as ET
from std_msgs.msg import String
from prost_ros.srv import StartPlanning, StartPlanningResponse, SubmitObservation, SubmitObservationResponse
from prost_ros.msg import KeyValue

class ProstBridge:
    def __init__(self):
        rospy.init_node('prost_bridge')

        self.prost_path = rospy.get_param('~prost_path', './prost.py')
        self.host = rospy.get_param('~host', '127.0.0.1')
        self.port = rospy.get_param('~port', 2323)
        self.rounds = rospy.get_param('~rounds', 100) # Number of rounds for the session
        self.time_allowed = rospy.get_param('~time_allowed', 1200) # Total time allowed
        self.ram_limit_kb = rospy.get_param('~ram_limit_kb', 1048576)

        self.parser_options = rospy.get_param('~parser_options', '-ipc2018 -fdrActions 0')
        self.search_engine = rospy.get_param(
            '~search_engine',
            '[Prost -s 1 -se [IPC2014 -t 1.0]]')
        self.search_engine = self._ensure_ram_limit(self.search_engine)

        self.server_socket = None
        self.client_socket = None
        self.proc = None
        self.lock = threading.Lock()
        
        self.current_domain = None
        self.current_instance = None
        self.next_action = None
        self.action_ready_event = threading.Event()
        self.proc_output_thread = None

        self.start_service = rospy.Service('~start_planning', StartPlanning, self.handle_start_planning)
        self.obs_service = rospy.Service('~submit_observation', SubmitObservation, self.handle_submit_observation)
        
        # Publisher for debug/info
        self.action_pub = rospy.Publisher('~action', String, queue_size=10)

        rospy.loginfo("ProstBridge initialized. Waiting for start_planning call.")

    def _ensure_ram_limit(self, search_engine):
        if re.search(r'(?<!\S)-ram\s+\d+', search_engine):
            rospy.loginfo("Using explicit PROST RAM limit from search_engine.")
            return search_engine

        ram_limit_kb = int(self.ram_limit_kb)
        patched = search_engine.replace('[Prost', f'[Prost -ram {ram_limit_kb}', 1)
        if patched == search_engine:
            rospy.logwarn(
                "Could not inject PROST RAM limit into search_engine '%s'. "
                "Continuing without an automatic RAM cap.",
                search_engine,
            )
            return search_engine

        rospy.loginfo(
            "Injected PROST RAM limit of %d KB into search_engine to reduce OOM kills.",
            ram_limit_kb,
        )
        return patched

    def _stream_prost_output(self):
        if not self.proc or not self.proc.stdout:
            return
        try:
            for line in self.proc.stdout:
                line = line.rstrip()
                if line:
                    rospy.logwarn(f"[PROST] {line}")
        except Exception as e:
            rospy.logwarn(f"Stopped reading PROST output: {e}")

    def start_server(self):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(1)
        rospy.loginfo(f"Listening on {self.host}:{self.port}")

    def handle_start_planning(self, req):
        rospy.loginfo("Received StartPlanning request.")
        self.current_domain = req.domain_content
        self.current_instance = req.instance_content
        timeout_val = req.timeout
        
        if self.proc:
            rospy.logwarn("Terminating existing PROST process.")
            self.proc.terminate()
            if self.client_socket:
                self.client_socket.close()

        if not self.server_socket:
            self.start_server()

        # Start PROST
        # We need to construct the task content for PROST to parse. 
        # Typically PROST parses domain and instance together. 
        # The Bridge sends the task description.
        # We'll concatenate domain and instance as the task description? 
        # Or does RDDLSim send them separately?
        # Looking at ipc_client.cc: executeParser(s). 
        # The string 's' is the decoded task description.
        # RDDL files usually contain both? Or we can concat.
        # Warning: Simple concatenation might not be valid RDDL if headers conflict, but 
        # usually one file imports the other or they are just blocks.
        # Let's assume we concat domain + \n + instance.
        self.task_content = req.domain_content + "\n" + req.instance_content

        # FIX: Parse the actual instance name from the RDDL content
        # Looks for: instance <name> {
        match = re.search(r'instance\s+(\w+)\s*\{', self.task_content)
        if match:
            instance_name = match.group(1)
            rospy.loginfo(f"Detected RDDL Instance Name: {instance_name}")
        else:
            instance_name = "unknown_instance"
            rospy.logwarn("Could not parse instance name from RDDL. Defaulting to 'unknown_instance'.")

        cmd = [self.prost_path, instance_name]

        if self.parser_options:
            cmd += ["--parser-options", self.parser_options]

        cmd += [
            "--hostname", self.host,
            "--port", str(self.port),
            self.search_engine
        ]

        timeout_match = re.search(r'(?<!\S)-t\s+([0-9]*\.?[0-9]+)', self.search_engine)
        if timeout_match:
            search_timeout = float(timeout_match.group(1))
            if search_timeout > 60.0:
                rospy.logwarn(
                    "Configured PROST search timeout is %.3fs per decision. "
                    "That is unusually high and may cause the planner to be killed "
                    "by memory or runtime limits on large domains.",
                    search_timeout,
                )
        
        rospy.loginfo(f"Starting PROST: {' '.join(cmd)}")
        
        # Verify binaries exist before running
        prost_root = os.path.dirname(os.path.abspath(self.prost_path))
        # prost.py defaults to release, checks for builds/release/rddl_parser/rddl-parser and builds/release/search/search
        # We can loosely check one of them to warn the user.
        # Note: If user uses --debug, prost.py looks in builds/debug. We assume release default here for the check.
        parser_bin = os.path.join(prost_root, "builds/release/rddl_parser/rddl-parser")
        search_bin = os.path.join(prost_root, "builds/release/search/search")
        
        if not os.path.exists(parser_bin) or not os.path.exists(search_bin):
            rospy.logerr(f"PROST binaries not found in {prost_root}/builds/release/. Did you verify the build completed successfully? (Run ./build.py in prost directory)")
            return StartPlanningResponse(False)

        try:
            # We must use absolute path for prost.py if not in PATH
            # Assuming prost_path is absolute or relative to CWD.
            # Ideally CWD should be the PROST root.
            workspace_dir = os.path.dirname(os.path.abspath(self.prost_path))
            self.proc = subprocess.Popen(
                cmd,
                cwd=workspace_dir,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
            self.proc_output_thread = threading.Thread(
                target=self._stream_prost_output,
                daemon=True,
            )
            self.proc_output_thread.start()
        except Exception as e:
            rospy.logerr(f"Failed to start PROST: {e}")
            return StartPlanningResponse(False)

        # Accept connection
        rospy.loginfo("Waiting for PROST to connect...")
        self.server_socket.settimeout(10.0) # 10s timeout for connection
        try:
            self.client_socket, addr = self.server_socket.accept()
            rospy.loginfo(f"Accepted connection from {addr}")
        except socket.timeout:
            rospy.logerr("Timed out waiting for PROST to connect.")
            return StartPlanningResponse(False)

        # Handle Session Init
        if not self.handle_session_init(timeout_val):
            return StartPlanningResponse(False)
            
        # Handle Round Init (Wait for round request)
        if not self.handle_round_init():
             return StartPlanningResponse(False)

        # Send Initial State (Wait for turn request? No, initRound sends turn)
        # We don't have the initial state here! 
        # The user calls StartPlanning, but we need the initial state to start the loop?
        # Actually, if we just send an empty state or default state, PROST might work?
        # BUT: The user probably wants to provide the initial state.
        # Let's assume the instance file defines the initial state (RDDL usually does).
        # We just need to send the 'turn' message with 'no-observed-fluents' if we rely on instance def?
        # Or we can parse the initial state from RDDL? Too complex.
        
        # Updated strategy: StartPlanning just sets up the session.
        # We wait for the FIRST submit_observation call to send the initial state.
        # BUT PROST logic: `initRound` -> `readState`. 
        # So PROST is blocked reading state immediately after round init.
        # So handle_start_planning CANNOT finish until we send the state.
        # THIS IS A DEADLOCK if start_planning blocks until init, but user can't call submit_observation until start_planning returns.
        
        # SOLUTION: StartPlanning should return success once Session is Init.
        # The `handle_round_init` and `send_initial_state` should happen in a separate thread OR
        # inside the first `submit_observation`.
        
        # But PROST sends `round-request` immediately after `session-init`.
        # So we can read `round-request` in `start_planning`.
        # Then we send `round-init`.
        # Then PROST waits for `readState`.
        # At this point, we return `StartPlanningResponse(True)`.
        # PROST is blocked on socket read.
        # User calls `submit_observation(initial_obs)`.
        # Handler sends state. PROST computes. PROST sends action. Handler returns action.
        
        return StartPlanningResponse(True)

    def handle_session_init(self, timeout):
        # Read session-request
        # <session-request><problem-name>...</problem-name>...</session-request>
        data = self.read_xml_message()
        rospy.logwarn(f"ROUND REQUEST: {data}")

        if not data:
            return False
        rospy.loginfo(f"Received session-request: {data}")
        
        # Send session-init
        # <session-init><task>BASE64</task><num-rounds>INT</num-rounds><time-allowed>INT</time-allowed></session-init>
        # Note: ipc_client.cc expects msg name "session-init"? 
        # Actually it checks dissect("task", s). 
        # AND it reads XMLNode. The name doesn't STRICTLY matter if dissect works, 
        # but standard is likely <session-init>. (Wait, client doesn't check root name in initSession, just dissects)
        
        encoded_task = base64.b64encode(self.task_content.encode('utf-8')).decode('utf-8')
        time_ms = int(self.time_allowed * 1000)
        # Prepare response
        # Using string formatting for simplicity.
        # Removing XML header as PROST's strxml.cc might not support processing instructions like <?xml ... ?>
        resp = f"""<session-init>
  <task>{encoded_task}</task>
  <num-rounds>{self.rounds}</num-rounds>
  <time-allowed>{time_ms}</time-allowed>
</session-init>\0"""
        rospy.logwarn(f"ROUND INIT SENDING: {resp}")
        self.send_xml_message(resp)
        return True

    def handle_round_init(self):
        # Read round-request
        data = self.read_xml_message()
        if not data: return False
        # rospy.loginfo(f"Received round-request: {data}")
        # root name should be round-request
        time_ms = int(self.time_allowed * 1000)
        # Send round-init
        # <round-init><time-left>...</time-left></round-init>
        resp = f"""<round-init>
<time-left>{time_ms}</time-left>
</round-init>\0"""
        self.send_xml_message(resp)
        return True

    def handle_submit_observation(self, req):
        # req.observation is KeyValue[]
        # req.reward is float
        
        # Construct <turn> message
        # <turn>
        #   <observed-fluent>
        #      <fluent-name>...</fluent-name>
        #      <fluent-arg>...</fluent-arg> ...
        #      <fluent-value>...</fluent-value>
        #   </observed-fluent> ...
        #   <time-left>...</time-left>
        #   <immediate-reward>...</immediate-reward>
        # </turn>
        rospy.loginfo("Prihvacena observacija")
        xml_parts = []
        time_ms = int(self.time_allowed * 1000)
        # xml_parts.append('<?xml version="1.0" encoding="UTF-8"?>')
        xml_parts.append('<turn>')
        xml_parts.append(f'<time-left>{time_ms}</time-left>') # Should technically decrease
        xml_parts.append(f'<immediate-reward>{req.reward}</immediate-reward>')
        
        if not req.observation:
            # Maybe start of round or no observations?
             xml_parts.append('<no-observed-fluents/>') # Check if PROST supports this tag or just empty
             # ipc_client.cc: if (node->size() == 2 && node->getChild(1)->getName() == "no-observed-fluents") assert(false);
             # Wait, that asset(false) suggests it CRASHES on no-observed-fluents?
             # "if size == 2 AND child is no-observed-fluents -> assert(false)"
             # This seems to verify it is NOT receiving that? Or maybe it handles observed-fluent loop?
             # Loop: for i=0 to size. if child name == observed-fluent.
             # So if we just have no children, it's fine.
             pass
        else:
            for kv in req.observation:
                # kv.key might be "fluent(arg1, arg2)"
                # We need to parse fluent name and args.
                # Simplified assumption: kv.key IS the full name including args? 
                # OR kv.key is name, args are separate?
                # RDDLSim usually assumes schema.
                # Let's try to parse "fluent(arg1, arg2)"
                if '(' in kv.key and kv.key.endswith(')'):
                    fname = kv.key.split('(')[0]
                    args = kv.key.split('(')[1][:-1].split(',')
                    args = [a.strip() for a in args]
                else:
                    fname = kv.key
                    args = []
                
                xml_parts.append('<observed-fluent>')
                xml_parts.append(f'<fluent-name>{fname}</fluent-name>')
                for arg in args:
                    xml_parts.append(f'<fluent-arg>{arg}</fluent-arg>')
                xml_parts.append(f'<fluent-value>{kv.value}</fluent-value>')
                xml_parts.append('</observed-fluent>')

        xml_parts.append('</turn>')
        xml_parts.append('\0')
        
        msg = "".join(xml_parts)
        #rospy.logwarn("=== SENDING TO PROST ===")
        #rospy.logwarn(msg)
        self.send_xml_message(msg)
        rospy.loginfo("Observacije poslate cekamo akcije")
        # Read Response (Actions)
        # <actions><action>...</action></actions>
        data = self.read_xml_message()
        #rospy.logwarn("=== RECEIVED FROM PROST ===")
        #rospy.logwarn(data)
        if not data:
            exit_code = self.proc.poll() if self.proc else None
            if exit_code is not None:
                if exit_code < 0:
                    rospy.logerr(
                        "PROST terminated by signal %d while waiting for an action. "
                        "This is often caused by an OOM kill; try lowering ~ram_limit_kb "
                        "or the search timeout.",
                        -exit_code,
                    )
                else:
                    rospy.logerr("PROST exited with code %d while waiting for an action.", exit_code)
            return SubmitObservationResponse(action_name="FAILED", action_params=[])
            
        # Parse XML
        # Remove null byte
        if data.endswith('\0'): data = data[:-1]
        rospy.loginfo("DATA:")
        print(data)
        rospy.loginfo(data)
        #try:
        root = ET.fromstring(data)
        if root.tag == "round-end":
            rospy.loginfo("Round ended.")
            return SubmitObservationResponse(action_name="ROUND_END", action_params=[])
        
        # Expect <actions>
        actions = root.findall('action')
        rospy.logwarn(f"Number of actions in PROST response: {len(actions)}")
        rospy.logwarn(f"Full PROST response root tag: {root.tag}")
        rospy.logwarn(f"Full PROST response: {ET.tostring(root, encoding='unicode')}")

        if not actions:
                return SubmitObservationResponse(action_name="NOOP", action_params=[])
        
        # Take first action (PROST usually returns one concurrent step, potentially multiple actions)
        # ROS wrapper simplifies to returning lists? Or just one?
        # User interface: action_name (string), action_params (string[])
        # If multiple actions, we might need a better msg. But usually in standard RDDL domains it's one action or factored.
        # Let's return the first one for now.
        rospy.loginfo("AKCIJE")
        rospy.loginfo(actions)
        act = actions[0]
        name = act.find('action-name').text
        args = [arg.text for arg in act.findall('action-arg')]
        
        self.action_pub.publish(f"{name}({','.join(args)})")
        
        return SubmitObservationResponse(action_name=name, action_params=args)

        #except ET.ParseError as e:
        #    rospy.logerr(f"XML Parse Error: {e}")
        #    return SubmitObservationResponse(action_name="ERROR", action_params=[])


    def read_xml_message(self):
        # Read until null byte or socket close
        # PROST messages end with \0
        buf = b""
        while True:
            try:
                chunk = self.client_socket.recv(4096)
                if not chunk:
                    exit_code = self.proc.poll() if self.proc else None
                    if exit_code is None:
                        rospy.logerr("PROST socket closed before sending a complete XML message.")
                    else:
                        if exit_code < 0:
                            rospy.logerr(
                                "PROST process was terminated by signal %d before sending a complete XML message.",
                                -exit_code,
                            )
                        else:
                            rospy.logerr(
                                "PROST process exited with code %d before sending a complete XML message.",
                                exit_code,
                            )
                    break
                buf += chunk
                if b'\0' in chunk:
                    break
            except Exception as e:
                rospy.logerr(f"Socket receive error: {e}")
                return None
        return buf.decode('utf-8').strip('\0')

    def send_xml_message(self, msg):
        try:
            self.client_socket.sendall(msg.encode('utf-8'))
        except Exception as e:
            rospy.logerr(f"Socket send error: {e}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = ProstBridge()
    node.run()
