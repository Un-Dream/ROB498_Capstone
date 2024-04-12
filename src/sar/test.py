grid_size = 5
step_size = 0.5
pattern = 'lawnmower'
height = 1
num_rows = grid_size
num_cols = grid_size
row_step = step_size
col_step = step_size
x = 0
y =0 
z = 0

for row in range(num_rows):
    for col in range(num_cols):
        x = col * col_step 
        if row % 2 == 0:
            x = col * col_step 
            y= row * row_step 
        else:
            x = (num_cols - col - 1) * col_step
            y = row * row_step 
        z = height
        print([x, y, z])