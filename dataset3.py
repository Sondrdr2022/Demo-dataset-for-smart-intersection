import pandas as pd
import matplotlib.pyplot as plt
import os

# Đường dẫn thư mục chứa các file CSV
folder = r"C:\Users\Admin\Downloads\sumo test\New folder"
filenames = [f"dataset{i+1}.csv" for i in range(4)]
labels = ["Hướng 1", "Hướng 2", "Hướng 3", "Hướng 4"]

plt.figure(figsize=(10,6))
for i, fname in enumerate(filenames):
    path = os.path.join(folder, fname)
    df = pd.read_csv(path, delimiter=';')
    plt.plot(df['step'], df['queue'], label=labels[i])

plt.xlabel('step (bước mô phỏng)')
plt.ylabel('queue (độ dài hàng đợi)')
plt.title('So sánh độ dài hàng đợi (queue) của 4 hướng giao thông')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()