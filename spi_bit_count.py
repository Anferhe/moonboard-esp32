safety_factor = 0.8

T0H = 0.5
T0L = 2.0
T1H = 1.2
T1L = 1.3

deviation = 0.15


T0 = T0H + T0L
T1 = T1H + T1L
deviation = deviation * safety_factor


# TODO Result deviation calculation
# x - count of high level bits for zero transmission
# y - count of high level bits for one transmission
def calculate(bits_per_data_bit: int):
    x_acceptable = list()
    y_acceptable = list()

    zero_time_per_bit = T0 / bits_per_data_bit
    one_time_per_bit = T1 / bits_per_data_bit

    for x in range(bits_per_data_bit + 1):
        x_checks = [
            zero_time_per_bit * x >= T0H - deviation,
            zero_time_per_bit * x <= T0H + deviation,
            zero_time_per_bit * (bits_per_data_bit - x) >= T0L - deviation,
            zero_time_per_bit * (bits_per_data_bit - x) <= T0L + deviation
        ]

        if all(x_checks):
            x_acceptable.append(x)

    for y in range(bits_per_data_bit + 1):
        y_checks = [
            one_time_per_bit * y >= T1H - deviation,
            one_time_per_bit * y <= T1H + deviation,
            one_time_per_bit * (bits_per_data_bit - y) >= T1L - deviation,
            one_time_per_bit * (bits_per_data_bit - y) <= T1L + deviation
        ]

        if all(y_checks):
            y_acceptable.append(y)

    return [(x, y) for x in x_acceptable for y in y_acceptable]


result = calculate(16)

for res in result:
    print(res)
