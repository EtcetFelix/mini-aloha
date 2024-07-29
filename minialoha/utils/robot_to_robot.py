from typing import List

DYN_RANGE_TICKS = 4096
DYN_RANGE_DEGREES = 360
HW_RANGE_TICKS = 1000
HW_RANGE_DEGREES = 240

DYN_TICKS_PER_DEGREE = (  # Number of dynamixel ticks in a degree
    DYN_RANGE_TICKS / DYN_RANGE_DEGREES
)
DYN_TICKS_IN_HW_RANGE = (  # Number of Dynamixel ticks in the Hiwonder degree range
    DYN_TICKS_PER_DEGREE * HW_RANGE_DEGREES
)

NORMALIZATION_VALUE = DYN_TICKS_IN_HW_RANGE / HW_RANGE_TICKS


def dynamixel_to_hiwonder_position(dynamixel_positions: List[int]) -> List[int]:
    """
    Translate a dynamixel position and return a hiwonder robot position.

    Dynamixel has a resolution of 4096 ticks/revolution.
    Hiwonder goes from -192 to 1193, but 'officially,' it goes from 0 to 1000.

    :param dynamixel_pos: List[int] - The current position of the dynamixel motor.
    """
    hiwonder_positions = []
    for pos in dynamixel_positions:
        # TODO: check if dynamixel goes to the position the nearest way (CW or CCW) or if it goes only 1 direction. For example, if you gave -1 with starting pos of 0, will it go all the way from 0 to 4096 or will it turn a single tick? If the former, need to somehow give a position in a way that it'll turn like the former, I wonder if it'll do it if you give it a negative number.
        # TODO: Since can't go from 240 to 360 degrees, need to decide what way to go when the starting position is near 0 but a negative dynamixel position is given. The hiwonder would jump from 0 to the max even though the dynamixel has only moved a single tick.
        pos = pos % DYN_RANGE_TICKS
        print(pos)
        # Disregard a position out of the range
        if pos < 0 or pos > DYN_TICKS_IN_HW_RANGE:
            hiwonder_positions.append(DYN_TICKS_IN_HW_RANGE)
            continue
        # Normalize the dynamixel position to the hiwonder range
        normalized_position = pos / NORMALIZATION_VALUE
        hiwonder_positions.append(normalized_position)

    # # Round the positions to convert to integers
    # for indx, pos in enumerate(hiwonder_positions):
    #     hiwonder_positions[indx] = round(pos)
    return hiwonder_positions


if __name__ == "__main__":
    # quick testing
    dyn_pos = [-1]
    print(dynamixel_to_hiwonder_position(dyn_pos))
