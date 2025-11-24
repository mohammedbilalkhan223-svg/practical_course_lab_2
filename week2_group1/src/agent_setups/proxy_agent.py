from mango import Agent, sender_addr

class ProxyAgent(Agent):
    """
    Forwards any messages received from addr1 to addr2 and vice versa.
    """
    def __init__(self, addr1, addr2):
        super().__init__()

        self.addr1 = addr1
        self.addr2 = addr2

    def handle_message(self, content, meta):
        sender = sender_addr(meta)

        if sender == self.addr1:
            self.schedule_instant_message(content, self.addr2)
        elif sender == self.addr2:
            self.schedule_instant_message(content, self.addr1)
