from typing import List, Union, NewType
import bluetooth
from bluetooth import BluetoothSocket, BluetoothError
from enum import Enum
from time import time, sleep
import rospy
from collections import namedtuple
import heapq as hp
from threading import Thread, Lock, Semaphore
from ctypes import c_ubyte
import socket
from ROS.message_server_publisher import MessageServerPublisher
from message_server_module.opcodes import ServerOpCode
from verysmall.srv import message_server_service

Message = namedtuple("Message", ["priority", "socket_id", "payload"])

Seconds = NewType('seconds', float)


import vision_module.sim.packet_pb2 as packet_pb2
import vision_module.sim.command_pb2 as command_pb2


class MessageServer:

    def __init__(self, owner_id: str = None, 
                 simulator_mode: bool = False,
                 max_sockets_capacity: int = 5,
                 max_queue_size: int = 5,
                 socket_timeout: Seconds = 0.00064):

        self._simulator_mode = simulator_mode
        self._capacity = max_sockets_capacity
        self._priority_queue = []
        self._max_queue_size = max_queue_size
        self.socket_timeout = socket_timeout

        self._adapter_lock = Lock()
        self._buffer_lock = Lock()
        self._socket_dic_lock = Lock()
        self._server_semaphore = Semaphore(0)

        self.TAG = "MESSAGE SERVER"

        self._sockets = {}
        self._num_active_sockets = 0
        self.default_port = 0x1001
        self._sockets_status = [0] * self._capacity
        suffix = '' if owner_id is None else '_' + owner_id
        self.service = rospy.Service('message_server_service' + suffix,
                                     message_server_service,
                                     self._service_request_handler)

        self.topic_publisher = MessageServerPublisher(owner_id)

        from ROS.message_server_subscriber import MessageServerSubscriber
        self.topic_subscriber = MessageServerSubscriber(self, owner_id)

        self._last_check = time()

        # ============================================
        UDP_IP = ""
        UDP_PORT = 20011

        self.sock = socket.socket(socket.AF_INET, # Internet
                            socket.SOCK_DGRAM) # UDP

        self.sock.connect((UDP_IP, UDP_PORT))

        # client, _ = sock.accept()

        self.message = packet_pb2.Packet()
        self.cmd = self.message.cmd.robot_commands.add()


    def _service_request_handler(self,
                                 req: message_server_service) -> int:
        response = ServerOpCode.ERROR
        if req.opcode == ServerOpCode.ADD.value:
            response = self._add_socket(req.socket_id, bytes(req.robot_mac_addr))

        elif req.opcode == ServerOpCode.REMOVE.value:
            response = self._remove_socket(req.socket_id)

        elif req.opcode == ServerOpCode.CHANGE_COLORS.value:
            response = self._change_team_color(req.socket_id)

        self.topic_publisher.publish(self._sockets_status)

        return response.value

    def _add_socket(self, socket_id: int, mac_address: bytes) -> ServerOpCode:
        response = ServerOpCode.ERROR

        if self._simulator_mode:
            self._sockets[socket_id] = ("fake_mac", None)
            self._num_active_sockets += 1
            self._update_socket_status(socket_id, ServerOpCode.ACTIVE)
            # response = ServerOpCode.OK
            return ServerOpCode.OK

        if self._num_active_sockets < self._capacity and \
                not self._mac_been_used(mac_address):

            mac_str = ":".join("%02x" % b for b in mac_address)

            sock = BluetoothSocket(bluetooth.RFCOMM)

            try:
                self._connect(sock, mac_str, 1)
                #sock.settimeout(self.socket_timeout)

                self._sockets[socket_id] = (mac_str, sock)

                self._num_active_sockets += 1
                self._update_socket_status(socket_id, ServerOpCode.ACTIVE)
                response = ServerOpCode.OK
            except IOError as e:
                sock.close()
                rospy.logfatal(self.TAG + ": ERROR IN ADDING SOCKET. " + repr(e))

        return response

    def _mac_been_used(self, mac_address: bytes) -> bool:
        for mac, _ in self._sockets.values():
            if mac == mac_address:
                return True
        return False

    def _update_socket_status(self, sock_id: int, status: Enum) -> None:

        if status == ServerOpCode.ACTIVE:
            self._sockets_status[sock_id] = 1
        else:
            self._sockets_status[sock_id] = 0

    def _remove_socket(self, socket_id: int) -> ServerOpCode:
        response = ServerOpCode.OK

        if self._simulator_mode:
            self._num_active_sockets -= 1
            self._update_socket_status(socket_id, ServerOpCode.INACTIVE)
            return response

        s = self._sockets.pop(socket_id, None)

        if s is None:
            response = ServerOpCode.ERROR
        else:
            self._num_active_sockets -= 1
            self._update_socket_status(socket_id, ServerOpCode.INACTIVE)
            socket = s[1]
            self._close(socket)
        return response


    def _change_team_color(self, socket_id: int) -> ServerOpCode:
        response = ServerOpCode.OK
        self.cmd.yellowteam = socket_id        
        return response

    def _lock_sockets(self):
        # self._socket_dic_lock.acquire()
        pass

    def _release_sockets(self):
        # self._socket_dic_lock.release()
        pass

    def loop(self) -> None:
        while not rospy.is_shutdown():
            self._server_semaphore.acquire()
            message = self._getItemFromBuffer()

            if message is not None:
                self.send_message(message.socket_id, message.payload)

            if time() - self._last_check > 1.0:
                self.topic_publisher.publish(self._sockets_status)
                self._last_check = time()

        self.on_shutdown()

    def _getItemFromBuffer(self) -> Message:
        m = None
        self._buffer_lock.acquire()
        if len(self._priority_queue) > 0:
            m = hp.heappop(self._priority_queue)
        self._buffer_lock.release()
        return m

    def putItemInBuffer(self, message: Message) -> None:
        self._buffer_lock.acquire()
        if len(self._priority_queue) == self._max_queue_size:
            self._priority_queue = hp.nsmallest(self._max_queue_size - 1,
                                                self._priority_queue)
            hp.heapify(self._priority_queue)
        hp.heappush(self._priority_queue, message)
        self._buffer_lock.release()
        self._server_semaphore.release()

    def send_message(self, id_: int, payload: List) -> None:
        released = False

        if id_ in self._sockets.keys():
            try:
                if self._simulator_mode:
                    self._sim_send(id_, payload)
                else:
                    self._send(self._sockets[id_][1], payload)
            
            except Exception as e:
                rospy.logfatal("TIMEOUT EXCEPTION " + repr(e))
                self._remove_socket(id_)

    def on_shutdown(self) -> None:
        for _, socket in self._sockets.values():
            self._close(socket)

    def _connect(self, sock: BluetoothSocket, mac: str, port: int) -> None:
        self._adapter_lock.acquire()
        try:
            sock.connect((mac, port))
        except:
            self._adapter_lock.release()
            raise IOError
        self._adapter_lock.release()

    def _send(self, sock: BluetoothSocket, payload: List) -> None:
        self._adapter_lock.acquire()
        try:
            n = sock.send(bytes(payload))
        except Exception as e:
            self._adapter_lock.release()
            raise e

        self._adapter_lock.release()

    def _sim_send(self, id_: int, payload: List):
        self._adapter_lock.acquire()
        self.cmd.id = id_        
        # rospy.logfatal(f"{payload[0]} {payload[1]} {payload[2]}")

        # self.LEFTFORWARD_RIGHTFORWARD = 0x00  # 0000 0000
        # self.LEFTFORWARD_RIGHTBACKWARD = 0x01  # 0000 0001
        # self.LEFTBACKWARD_RIGHTFORWARD = 0x02  # 0000 0010
        # self.LEFTBACKWARD_RIGHTBACKWARD = 0x03  # 0000 0011

        self.cmd.wheel_left =  -payload[1] if (payload[0]&2) else payload[1] 
        self.cmd.wheel_right = -payload[2] if (payload[0]&1) else payload[2]

        payload = self.message.SerializeToString()
        self.sock.sendall(payload)
        self._adapter_lock.release()
        # rospy.logfatal(f"{id_} {payload[1]}")

    def _close(self, sock: BluetoothSocket) -> None:
        rospy.logfatal("REMOVING SOCKET: " + repr(sock))
        self._adapter_lock.acquire()
        sock.close()
        sleep(self.socket_timeout)
        self._adapter_lock.release()
