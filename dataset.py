import traci
import csv

SUMO_CFG = r"C:\\Users\\Admin\\Downloads\\sumo test\\dataset.sumocfg"
base_output = r"C:\\Users\\Admin\\Downloads\\sumo test\\dataset"
input_edges = ["E1-3", "E2-3", "E4-3", "E5-3"]

if __name__ == "__main__":
    traci.start(["sumo", "-c", SUMO_CFG])
    csvfiles = []
    writers = []
    # Tạo 4 file và 4 writer, lưu vào list
    for i in range(4):
        csvfile = open(f"{base_output}{i+1}.csv", 'w', newline='')
        writer = csv.writer(csvfile, delimiter=';')
        writer.writerow([
            "step", "queue", "waitingTime", "avgSpeed", "density", "outflow", "greenTime", "redTime", "cycleTime"
        ])
        csvfiles.append(csvfile)   # <--- thêm vào list
        writers.append(writer)     # <--- thêm vào list
    for step in range(600):
        traci.simulationStep()
        if step % 30 != 0:
            continue
        tlsID = traci.trafficlight.getIDList()[0]
        logic = traci.trafficlight.getCompleteRedYellowGreenDefinition(tlsID)[0]
        phases = logic.getPhases()
        greenTime = sum(ph.duration for ph in phases if "G" in ph.state)
        redTime = sum(ph.duration for ph in phases if "r" in ph.state or "R" in ph.state)
        cycleTime = sum(ph.duration for ph in phases)
        # Ghi cho từng hướng vào từng file
        for i, edge in enumerate(input_edges):
            queue = traci.lane.getLastStepHaltingNumber(edge + "_0")
            waitTime = traci.lane.getWaitingTime(edge + "_0")
            avgSpeed = traci.lane.getLastStepMeanSpeed(edge + "_0")
            total_vehicles = traci.lane.getLastStepVehicleNumber(edge + "_0")
            length = traci.lane.getLength(edge + "_0")
            density = total_vehicles / length if length > 0 else 0
            outflow = total_vehicles
            writers[i].writerow([
                step,
                queue,
                waitTime,
                str(f"{avgSpeed:.6f}").replace('.', ','),  # Đổi dấu chấm thành dấu phẩy
                str(f"{density:.6f}").replace('.', ','),
                outflow,greenTime,redTime,cycleTime
                ])
    # Đóng tất cả file
    for csvfile in csvfiles:
        csvfile.close()
    traci.close()