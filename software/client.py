import asyncio
import threading
import websockets
import json


class Client:
    def __init__(self):
        self.run = False
        self.blue = True
        self.robot = ('GibiBot')
        self.host = ('192.168.3.40')
        self.port = ('8222')

    async def listen(self):
        print("Connecting to " + str(self.host) + " on port " + str(self.port))
        uri = "ws://" + str(self.host) + ":" + str(self.port)
        async with websockets.connect(uri) as ws:
            while True:
                msg = await ws.recv()
                cmd = json.loads(msg)
                print("Received message from server: " + str(cmd))
                try:
                    self.process_command(cmd)
                except ValueError:
                    continue

    def process_command(self, cmd):
        if self.robot in cmd["targets"]:
            if cmd["signal"] == "changeID" and self.robot != cmd["new_robot_id"]:
                new_robot_id = cmd["new_robot_id"]
                try:
                    self.robot = new_robot_id
                except ValueError:
                    print("Invalid robot id")
            elif cmd["signal"] == "stop":
                self.run = False
            elif cmd["signal"] == "start":
                color = cmd["baskets"][cmd["targets"].index(self.robot)]
                if color == "blue":
                    self.blue = True
                else:
                    self.blue = False
                self.run = True

    def get_current_referee_command(self):
        return self.run, self.blue

    def start_loop(self):
        loop = asyncio.new_event_loop()
        loop.run_until_complete(self.listen())
        loop.run_forever()

    def start(self):
        t = threading.Thread(target=self.start_loop)
        t.start()