class distance_data_set:

    def get_dest_addr(self):
        return self.dest_addr

    def get_source_addr(self):
        return self.source_addr

    def set_distance(self, distance):
        self.distance_numbers.append(distance)

    def get_distances(self):
        return self.distance_numbers

    def get_distance_size(self):
        return len(self.distance_numbers)

    def get_distance_norm(self):
        return sum(self.distance_numbers) / len(self.distance_numbers)

    def set_distance_bias(self, distance_bias):
        self.distance_bias_numbers.append(distance_bias)

    def get_distances_bias(self):
        return self.distance_bias_numbers

    def get_distance_bias_size(self):
        return len(self.distance_bias_numbers)

    def get_distance_bias_norm(self):
        return sum(self.distance_bias_numbers) / len(self.distance_bias_numbers)


    def __init__(self, dest_addr, source_addr):
        self.dest_addr = dest_addr
        self.source_addr = source_addr
        self.distance_numbers = []
        self.distance_bias_numbers = []