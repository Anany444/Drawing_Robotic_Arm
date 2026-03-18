import matplotlib.pyplot as plt

# File path (IMPORTANT: use correct directory)
file_path = "coords.txt"   # same folder OR give full path

# Read file
with open(file_path, "r") as f:
    content = f.read()

# Split and convert to float
nums = list(map(float, content.split()))

# Safety check
if len(nums) % 3 != 0:
    raise ValueError("Data is not in (x, y, z) triplets!")

# Extract x, y
x = nums[0::3]
y = nums[1::3]

# Plot
fig, ax = plt.subplots()
ax.plot(x, y, marker='.')

# Axis formatting
ax.spines['left'].set_position('zero')
ax.spines['bottom'].set_position('zero')
ax.spines['right'].set_color('none')
ax.spines['top'].set_color('none')

ax.set_aspect('equal', adjustable='box')
ax.grid(True)

plt.show()