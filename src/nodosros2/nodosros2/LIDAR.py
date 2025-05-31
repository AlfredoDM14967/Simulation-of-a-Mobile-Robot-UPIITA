from std_msgs.msg import String

Msg = String()
Msg = "1 2 3"

Sep = Msg.split()
print(int(Sep[2])+5)