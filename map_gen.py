import random

MIN_ROOM_WIDTH = 2
MAX_REC_DEPTH = 1

def bsp(map, l, r, t, b, depth, phase):
    if depth == 0:
        # split_l = random.randrange(l, l + (r-l) / 2)
        # split_r = random.randrange(split_l, r)
        # split_t = random.randrange(l, r)
        # split_b = random.randrange(l, r)

        for i in range(l+1, r-1):
            for j in range(t+1, b-1):
                map[i][j] = 0

        return map

    if (phase):
        split = random.randrange(l, r)
        m = bsp(map, l, split, t, b, depth - 1, not phase)
        for i in range(l, split):
            for j in range(t, b):
                map[i][j] = m[i][j]

        m = bsp(map, split, r, t, b, depth - 1, not phase)
        for i in range(split, r):
            for j in range(t, b):
                map[i][j] = m[i][j]
    else:
        split = random.randrange(t, b)
        m = bsp(map, l, r, t, split, depth - 1, not phase)
        for i in range(l, r):
            for j in range(t, split):
                map[i][j] = m[i][j]

        m = bsp(map, l, r, split, b, depth - 1, not phase)
        for i in range(l, r):
            for j in range(split, b):
                map[i][j] = m[i][j]

    return map


if __name__ == "__main__":

    random.seed(64)
    map = [[1] * 40] * 40
    for i in map:
        print(i)

    map = bsp(map, 0, 40, 0, 40, MAX_REC_DEPTH, True)
    for i in map:
        print("".join([str(x) for x in i]))