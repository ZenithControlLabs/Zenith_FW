import csv
import math
import matplotlib.pyplot as plt

STRETCH_X = 1.0
STRETCH_Y = 1.5
CENTER = (-0.09*STRETCH_X,-0.03*STRETCH_Y)
REF = [(-0.95, -0.02),(-0.79, -0.76),(-0.07, -0.97),(0.57, -0.77),(0.73, -0.03),(0.55, 0.67),(-0.08, 0.92),(-0.7600, 0.6900)]
REF = [(x*STRETCH_X,y*STRETCH_Y) for (x,y) in REF]

THRESH = math.pi/180. * 5. # 5 better with local_maxima
THRESH1 = math.pi/180. * 1.
WINDOW = 7
LPF = 100

def mag(x,y):
    return math.sqrt(x*x + y*y)

def lpf(dt, prev, cur):
    alpha = dt / ((1/(math.tau * LPF)) + dt)
    return prev + alpha * (cur - prev)

def lpf2(dt, prev2, cur2):
    (p0,p1) = prev2
    (c0,c1) = cur2
    return (lpf(dt,p0,c0),lpf(dt,p1,c1))

def local_maxima(stream):
    gate = [(0,0)] * 8
    free = [True] * 8 
    # window size probably doesnt matter all that much
    readings = [(0,0)] * WINDOW # 0 is oldest, WINDOW-1 is newest
    (t0,_,_) = next(stream)
    for (t,x,y) in stream:
        dt = t - t0
        if dt == 0: continue
        t0 = t

        (x,y) = (x - CENTER[0], y - CENTER[1])
        p = (mag(x,y),math.atan2(y,x))
        p = lpf2(dt, readings[-1], p)

        readings.pop(0)
        readings.append(p)

        (r,th) = readings[WINDOW//2]

        ok = True
        for (i,(rv,thv)) in enumerate(gate):
            if (not free[i]) and (abs(th - thv) < THRESH) :
                if r > rv:
                    free[i] = True
                else:
                    ok = False # there is a greater vertex already
                # break ?  
        drl0 = (r - readings[2][0]) > 0
        drl1 = (readings[2][0] - readings[1][0]) > 0
        drl2 = (readings[1][0] - readings[0][0]) > 0
        drr0 = (readings[-3][0] - r) < 0
        drr1 = (readings[-2][0] - readings[-3][0]) < 0
        drr2 = (readings[-1][0] - readings[-2][0]) < 0

        if ok and drl0 and drl1 and drr0 and drr1: # Local maximum vertex without greater radius at angle
            # assert ( none of the gate points are within THRESH of p )

            i = 0
            while i <= 7 and not (free[i]) :
                i += 1
            if i > 7:
                print(f"dropped gate point {r},{th}")
            else:
                free[i] = False
                gate[i] = (r,th)
    return gate

def julien_algorithms(stream):
    gate16 = [0] * 16
    for (_,x,y) in stream:
        (x,y) = (x - CENTER[0], y - CENTER[1])
        (r,th) = (mag(x,y),math.atan2(y,x))

        th_b_i = int(round(th/(math.tau) * 16)) % 16
        th_b = th_b_i * math.tau/16
        if abs((th_b - th + math.pi) % math.tau - math.pi) < THRESH1:
            gate16[th_b_i] = max(gate16[th_b_i],r)
    # TODO
    return gate16



if __name__ == "__main__":
    gate_xy = []
    with open("fw/platforms/phobri64/utils/ctlr.csv") as f:
        r = csv.reader(f)
        next(r)
        gate = local_maxima(map(lambda row: (float(row[0])/1e6,float(row[1]) * STRETCH_X,float(row[2]) * STRETCH_Y), r))
        gate.sort(key=(lambda p: p[1]))

        print("Gate points:")
        for (i,(r,th)) in enumerate(gate):
            x = r * math.cos(th) + CENTER[0]
            y = r * math.sin(th) + CENTER[1]
            print(f"{i} : {x} , {y}")
            gate_xy.append((x,y))
    gate16_xy = []
    with open("fw/platforms/phobri64/utils/ctlr.csv") as f:
        r = csv.reader(f)
        next(r)
        gate16 = julien_algorithms(map(lambda row: (float(row[0])/1e6,float(row[1]) * STRETCH_X,float(row[2]) * STRETCH_Y), r))
        for (i,r) in enumerate(gate16):
            th = i * math.tau / 16
            x = r * math.cos(th) + CENTER[0]
            y = r * math.sin(th) + CENTER[1]
            gate16_xy.append((x,y))
    

    # Extract coordinates
    x = [p[0] for p in REF]
    y = [p[1] for p in REF]

    x_add = [p[0] for p in gate_xy]
    y_add = [p[1] for p in gate_xy]

    x_add16 = []
    y_add16 = []
    #x_add16 = [p[1] for p in gate16_xy]
    #y_add16 = [p[2] for p in gate16_xy]
    with open("fw/platforms/phobri64/utils/ctlr.csv") as f:
        r = csv.reader(f)
        next(r)
        for row in r:
            x_add16.append(float(row[1]) * STRETCH_X)
            y_add16.append(float(row[2]) * STRETCH_Y)


    # Create plot
    plt.figure(figsize=(6, 6))
    plt.axis('equal')

    # Plot the octagon (connect back to first point)
    plt.plot(x + [x[0]], y + [y[0]], 'k-', linewidth=2)
    plt.scatter(x, y, color='red', s=50)


    plt.scatter(x_add, y_add, color='green',
            edgecolors='darkgreen', linewidth=0, zorder=5, 
            label='Additional points (not connected)')
    
    plt.scatter(x_add16, y_add16, color='gray',
            edgecolors='blue', linewidth=0, zorder=-1, 
            label='Additional points (not connected)')

    # Add simple labels
    #for i, (xi, yi) in enumerate(gate_xy):
    #    plt.text(xi, yi, f'({xi:.1f},{yi:.1f})', 
    #            ha='center', va='bottom', fontsize=8)

    plt.title('Octagon')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.grid(True, alpha=0.3)
    plt.show()