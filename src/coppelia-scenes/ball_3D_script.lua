function sysCall_init()
    sphere = sim.getObjectHandle(sim.handle_self)
    lViz = sim.getObjectHandle('Vision_Sensor_Left')
    lMotor = sim.getObjectHandle('Left_Motor1')
    
    t_ref = sim.getSimulationTime()
    direction = -1
    r = 1.5
    theta = 0
    v = 0.0005
    
    dist = 0
end

function sysCall_actuation()
    t = sim.getSimulationTime()
    theta = theta + direction * v
    
    posS = sim.getObjectPosition(sphere, -1)
    xs, ys, zs = posS[1], posS[2], posS[3]
    xs = r * math.cos(theta)
    ys = r * math.sin(theta)
    sim.setObjectPosition(sphere, -1, {xs, ys, zs})
    
    local theta_m = sim.getJointPosition(lMotor)
    local theta_s = sphere_angle(xs, ys, zs)
    local dist = compute_position_reward(theta_m, theta_s)

    local reward_table = {}
    
    local reward = gaussian(100, 0, 0.4, 4*dist)
    
    reward_table.data = reward
    sim.setStringSignal("reward", sim.packTable(reward_table))

    local ran = math.random()
    if dist > 0.2 or dist < -0.2 or t - t_ref > 15 then
        t_ref = t
        if ran >= 0.5 then
            theta = sim.getJointPosition(lMotor) + ran / 5
            direction = -1
        else
            theta = sim.getJointPosition(lMotor) - ran / 5
            direction = 1
        end
    end
end

function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()
    -- do some clean-up here
end

function sphere_angle(xs, ys, zs)
    local theta_s
    if xs > 0 then
        theta_s = math.atan(ys / xs)
    elseif ys > 0 then
        theta_s = math.pi / 2 - math.atan(xs / ys)
    elseif ys < 0 then
        theta_s = -math.pi / 2 - math.atan(xs / ys)
    elseif xs < 0 then
        theta_s = math.atan(ys / xs) + math.pi
    else
        theta_s = 0
    end
    return theta_s
end

function compute_position_reward(theta_m, theta_s)
    local dist = 0
    if theta_m >= 0 and theta_s >= 0 then
        dist = theta_m - theta_s
    elseif theta_m < 0 and theta_s < 0 then
        dist = theta_m - theta_s
    elseif theta_m < 0 and theta_s > 0 then
        if theta_s - theta_m < math.pi then
            dist = -(theta_s - theta_m)
        else
            dist = 2*math.pi - (theta_s - theta_m)
        end
    elseif theta_m >= 0 and theta_s <= 0 then
        if theta_m - theta_s < math.pi then
            dist = theta_m - theta_s
        else
            dist = -2*math.pi + (theta_m - theta_s)
        end
    end
    return dist
end

function gaussian(a, b, c, x)
    return a * math.exp(-0.5 * (x - b)^2 / c^2)
end
