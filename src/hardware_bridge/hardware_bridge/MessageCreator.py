from geometry_msgs.msg import Quaternion

from collections import deque

class MessageCreator():
    def __init__(self) -> None:
        self.raw_msgs_ = deque()

    def Translate(self, raw_msg):
        self.raw_msgs_.append(raw_msg)

    def Spin(self):
        pass


class Identifier():
    import yaml
    from ament_index_python.packages import get_package_share_directory
    table_ = {}
    def __init__(self) -> None:
        #read from yaml file to populate table
        share_directory = get_package_share_directory('hardware_bridge')

        with open(share_directory + "/config/msg_types.yaml") as config_file:
            self.table_ = yaml.safe_load(config_file)

    def Ideentify(self, raw_msg) -> str:
        # look at first byte to see what the message is
        id = raw_msg[0]

        # read from yaml file to find out what the message type should be
        return self.table_[id]["type"]
        

