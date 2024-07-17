import random

MIN_ROOM_WIDTH = 2
MAX_REC_DEPTH = 4

def bsp(map, l, r, t, b, depth):
    if depth == 0:
        if (r-MIN_ROOM_WIDTH <= l+MIN_ROOM_WIDTH or b-MIN_ROOM_WIDTH <= t+MIN_ROOM_WIDTH):
            return map

        split_l = l + MIN_ROOM_WIDTH
        split_r = r - MIN_ROOM_WIDTH
        split_t = t + MIN_ROOM_WIDTH
        split_b = b - MIN_ROOM_WIDTH

        print(f"{l} {r} {t} {b}")
        for j in range(split_t, split_b):
            for i in range(split_l, split_r):
                map[i][j] = 0
        print(f"({split_l}, {split_b}) -- ({split_l}, {split_t})")
        print(f"({split_l}, {split_t}) -- ({split_r}, {split_t})")
        print(f"({split_r}, {split_t}) -- ({split_r}, {split_b})")
        print(f"({split_r}, {split_b}) -- ({split_l}, {split_b})")
        return map

    if (r - l >= b - t):
        if r-l < MIN_ROOM_WIDTH:
            return map
        
        split = random.randrange(l, r)
        if split - l > MIN_ROOM_WIDTH:
            m = bsp(map, l, split, t, b, depth - 1)
            for i in range(l, split):
                for j in range(t, b):
                    map[i][j] = m[i][j]

        if r - split > MIN_ROOM_WIDTH: 
            m = bsp(map, split, r, t, b, depth - 1)
            for i in range(split, r):
                for j in range(t, b):
                    map[i][j] = m[i][j]
    else:
        if b-t < MIN_ROOM_WIDTH:
            return map

        split = random.randrange(t, b)
        if split - t > MIN_ROOM_WIDTH:
            m = bsp(map, l, r, t, split, depth - 1)
            for i in range(l, r):
                for j in range(t, split):
                    map[i][j] = m   [i][j]

        if b - split > MIN_ROOM_WIDTH:
            m = bsp(map, l, r, split, b, depth - 1)
            for i in range(l, r):
                for j in range(split, b):
                    map[i][j] = m[i][j]

    return map


if __name__ == "__main__":

    map = [[1] * 40 for i in range(40)]
    map = bsp(map, 0, 40, 0, 40, MAX_REC_DEPTH)
    for i in map:
        print("".join(['1' if x == 1 else ' ' for x in i]))