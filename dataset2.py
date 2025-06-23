import pandas as pd

# Đọc dữ liệu như bạn đã làm
dfs = [
    pd.read_csv(r"C:\Users\Admin\Downloads\sumo test\New folder\dataset1.csv", delimiter=';'),
    pd.read_csv(r"C:\Users\Admin\Downloads\sumo test\New folder\dataset2.csv", delimiter=';'),
    pd.read_csv(r"C:\Users\Admin\Downloads\sumo test\New folder\dataset3.csv", delimiter=';'),
    pd.read_csv(r"C:\Users\Admin\Downloads\sumo test\New folder\dataset4.csv", delimiter=';')
]

# Số bước cuối cùng dùng để tính trung bình (có thể thay đổi)
num_last_steps = 5

# Lấy cycleTime và greenTime hiện tại (giả sử giống nhau ở các hướng)
cycle_time = dfs[0]['cycleTime'].iloc[-1]
num_directions = 4

# Tính toán chỉ số trung bình trên các bước cuối cho mỗi hướng
stats = []
for i, df in enumerate(dfs):
    last = df.tail(num_last_steps)
    avg_queue = last['queue'].mean()
    avg_waiting = last['waitingTime'].mean()
    avg_outflow = last['outflow'].mean()
    stats.append({'dir': i+1, 'avg_queue': avg_queue, 'avg_waiting': avg_waiting, 'avg_outflow': avg_outflow})

# Tính điểm ùn tắc (có thể điều chỉnh trọng số cho phù hợp mục tiêu)
for s in stats:
    s['congestion_score'] = s['avg_queue'] + s['avg_waiting'] - s['avg_outflow']

# Phân bổ lại greenTime dựa trên điểm ùn tắc
scores = [max(0.1, s['congestion_score']) for s in stats]
total_score = sum(scores)
green_times_new = [round(cycle_time * sc / total_score, 2) for sc in scores]

# Đảm bảo tổng greenTime không vượt quá cycle_time
sum_green = sum(green_times_new)
if sum_green > cycle_time:
    diff = sum_green - cycle_time
    for i in range(num_directions):
        green_times_new[i] -= diff / num_directions

# In kết quả
for i, gt in enumerate(green_times_new):
    print(f"Hướng {i+1}: greenTime mới = {gt:.2f} giây")