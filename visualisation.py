SQUARE_SIZE = 500

corners = [(0, 0), (SQUARE_SIZE, 0), (SQUARE_SIZE, SQUARE_SIZE), (0, SQUARE_SIZE)]
lines = []


for i in range(1, len(corners) + 1):
    lines.append(corners[i - 1] + corners[i % len(corners)])

for line in lines:
    print("drawLine:", str(line))
