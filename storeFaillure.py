

class info_storage():
    def __init__(self):
        self.info = []
        self.info.append("")

    def add_info(self, info):
        self.info.append(info)

    def get_info(self):
        return self.info

    def clear_info(self):
        self.info = []
        self.info.append("")
        
    def write_info_to_file(self, file_name):
        with open(file_name, 'w') as f:
            for item in self.info:
                f.write("%s\n" % item)
        self.clear_info()
        
        