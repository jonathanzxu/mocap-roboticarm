import time
import math
import numpy as np


def normalize(v):
    norm = np.sqrt(np.sum(v**2))
    if norm == 0: 
       return v
    return v / norm

#markers = 2d list with double xyz positions of each marker
def calculateangles(markers):
    m0 = np.array(markers[0], dtype=float)
    m1 = np.array(markers[1], dtype=float)
    m2 = np.array(markers[2], dtype=float)
    m3 = np.array(markers[3], dtype=float)
    m4 = np.array(markers[4], dtype=float)
    m5 = np.array(markers[5], dtype=float)
    m6 = np.array(markers[6], dtype=float)
    m7 = np.array(markers[7], dtype=float)
    m8 = np.array(markers[8], dtype=float)
    m9 = np.array(markers[9], dtype=float)
    m10 = np.array(markers[10], dtype=float)
    m11 = np.array(markers[11], dtype=float)
    #generate rotation matrices with respect to global coordinate system
    x1 = m0 - m1
    v1 = m2 - m1
    y1 = np.cross(x1, v1)
    z1 = np.cross(x1, y1)
    y2 = m5 - m4
    v2 = m3 - m4
    z2 = np.cross(v2, y2)
    x2 = np.cross(y2, z2)
    y3 = m7 - m8
    v3 = m7 - m6
    z3 = np.cross(v3, y3)
    x3 = np.cross(y3, z3)
    x4 = m10 - m9
    v4 = m11 - m9
    z4 = np.cross(v4, x4)
    y4 = np.cross(x4, z4)
    x1 = normalize(x1)
    y1 = normalize(y1)
    z1 = normalize(z1)
    x2 = normalize(x2)
    y2 = normalize(y2)
    z2 = normalize(z2)
    x3 = normalize(x3)
    y3 = normalize(y3)
    z3 = normalize(z3)
    x4 = normalize(x4)
    y4 = normalize(y4)
    z4 = normalize(z4)
    #notation: Rab = rotation matrix of b from a; g = global
    #print(x1)
    Rg1 = np.transpose(np.array([x1, y1, z1]))
    Rg2 = np.transpose(np.array([x2, y2, z2]))
    Rg3 = np.transpose(np.array([x3, y3, z3]))
    Rg4 = np.transpose(np.array([x4, y4, z4]))
    print(Rg1)
    print(Rg2)
    print(Rg3)
    print(Rg4)
    #new rot matrices
    
    R12 = np.matmul(np.linalg.inv(Rg1), Rg2)
    R23 = np.matmul(np.linalg.inv(Rg2), Rg3)
    R34 = np.matmul(np.linalg.inv(Rg3), Rg4)
    #print(R12)
    #print(R23)
    #print(R34)

    #calculate euler angles
    '''
    alphag1 = math.degrees(np.arctan(Rg1[1, 0]/Rg1[0, 0]))
    betag1 = math.degrees(np.arcsin(-1 * Rg1[2, 0]))
    gammag1 = math.degrees(np.arctan(Rg1[2, 1]/Rg1[2, 2]))
    
    alpha12 = math.degrees(np.arctan(R12[1, 0]/R12[0, 0]))
    beta12 = math.degrees(np.arcsin(-1 * R12[2, 0]))
    gamma12 = math.degrees(np.arctan(R12[2, 1]/R12[2, 2]))

    

    alpha23 = math.degrees(np.arctan(R23[1, 0]/R23[0, 0]))
    beta23 = math.degrees(np.arcsin(-1 * R23[2, 0]))
    gamma23 = math.degrees(np.arctan(R23[2, 1]/R23[2, 2]))
    
    alpha34 = math.degrees(np.arctan(R34[1, 0]/R34[0, 0]))
    beta34 = math.degrees(np.arcsin(-1 * R34[2, 0]))
    gamma34 = math.degrees(np.arctan(R34[2, 1]/R34[2, 2]))
    '''

    betag1 = np.arctan2(-1*Rg1[2,0], np.sqrt(Rg1[0,0]**2 + Rg1[1,0]**2))
    alphag1 = math.degrees(np.arctan2(Rg1[1,0]/np.cos(betag1), Rg1[0,0]/np.cos(betag1)))
    gammag1 = math.degrees(np.arctan2(Rg1[2,1]/np.cos(betag1), Rg1[2,2]/np.cos(betag1)))
    betag1 = math.degrees(betag1)

    beta12 = np.arctan2(-1*R12[2,0], np.sqrt(R12[0,0]**2 + R12[1,0]**2))
    alpha12 = math.degrees(np.arctan2(R12[1,0]/np.cos(beta12), R12[0,0]/np.cos(beta12)))
    gamma12 = math.degrees(np.arctan2(R12[2,1]/np.cos(beta12), R12[2,2]/np.cos(beta12)))
    beta12 = math.degrees(beta12)

    beta23 = np.arctan2(-1*R23[2,0], np.sqrt(R23[0,0]**2 + R23[1,0]**2))
    alpha23 = math.degrees(np.arctan2(R23[1,0]/np.cos(beta23), R23[0,0]/np.cos(beta23)))
    gamma23 = math.degrees(np.arctan2(R23[2,1]/np.cos(beta23), R23[2,2]/np.cos(beta23)))
    beta23 = math.degrees(beta23)

    beta34 = np.arctan2(-1*R34[2,0], np.sqrt(R34[0,0]**2 + R34[1,0]**2))
    alpha34 = math.degrees(np.arctan2(R34[1,0]/np.cos(beta34), R34[0,0]/np.cos(beta34)))
    gamma34 = math.degrees(np.arctan2(R34[2,1]/np.cos(beta34), R34[2,2]/np.cos(beta34)))
    beta34 = math.degrees(beta34)
    
    print(f"1 to 2 angles - alpha: {alpha12}° beta: {beta12}° gamma: {gamma12}°")
    print(f"2 to 3 angles - alpha: {alpha23}° beta: {beta23}° gamma: {gamma23}°")
    print(f"3 to 4 angles - alpha: {alpha34}° beta: {beta34}° gamma: {gamma34}°")

    print(f"Approximate angles: Shoulder Rotated ≈ {int(gamma12)}° - Shoulder Tilted ≈ {int(alpha12)}° - Elbow Tilted ≈ {int(alpha23)}° - Wrist Tilted ≈ {int(alpha34 if alpha34 > 0 else (alpha34 + 360))}° - Wrist Rotated ≈ {-1*int(gamma34)}°")

t1 = time.time()
#calculateangles(markers=[[1,23,13],[41,54,26],[73,82,96],[10,51,15],[23,74,5],[1,127,7],[91,23,28],[22,23,222],[65,66,37],[21,26,34],[41,13,43],[142,15,3]])
#calculateangles(markers=[[237.469,538.2484,254.8779],[280.497,539.3055,256.3565],[324.6915,538.8788,224.4204],[239.0268,616.2815,211.7472],[211.6873,613.0554,210.0513],[209.7667,629.001,210.0268],[128.75,664.1906,204.8625],[152.6109,666.5114,207.1383],[153.0074,648.3954,206.7344],[338.3122,603.2166,211.2846],[325.2154,610.0056,210.8075],[347.5862,618.0909,212.499]])
#calculateangles(markers=[[236.35928,538.98291,255.56940],[278.69806,539.33105,255.96320],[322.84949,538.51648,223.69504],[252.15396,651.03442,211.79474],[254.61957,678.30713,211.36392],[270.69766,677.11023,212.58344],[342.89136,779.27734,216.52682],[366.61166,778.51331,218.79251],[364.72479,760.30481,218.70514],[563.86957,734.99927,227.02441],[549.69458,736.79089,225.25195],[566.19641,754.82532,225.89313]])
#calculateangles(markers=[[237.79459,538.68701,254.50452],[280.30945,539.25366,254.37634],[324.16745,538.63074,220.94815],[237.05121,636.90002,210.05215],[217.80972,656.32520,208.99777],[228.68262,668.38098,210.42166],[206.03163,781.37427,209.22482],[228.37698,789.73730,210.80280],[233.64433,772.44983,210.79822],[411.14716,751.48254,214.07916],[399.33115,760.46484,214.13751],[422.41589,765.68762,204.37042]])
calculateangles(markers=[[237.95129,539.05713,254.59796],[280.12015,539.09631,254.02396],[323.65869,537.96814,221.01425],[267.55090,650.98816,209.35947],[270.60748,677.98193,207.33368],[285.32358,676.37085,201.21796],[350.76404,784.96106,167.38675],[371.45337,792.90796,157.86417],[375.79831,775.43750,155.61734],[544.65442,838.89502,64.33301],[531.67023,834.34363,69.88287],[534.52148,856.80640,59.35016]])
#calculateangles(markers=[[238.07745,538.95007,254.51794],[280.18076,539.05591,253.99643],[324.39017,538.91223,221.62515],[265.46014,648.91003,213.62671],[286.39468,666.09949,219.89914],[295.73566,653.16919,222.76999],[404.64108,660.53149,256.89105],[408.64694,636.96765,259.22641],[391.53586,634.80139,253.26544],[489.18369,475.83630,274.40076],[476.99002,486.30981,272.64279],[500.10574,488.00769,263.78458]])
t2 = time.time()
print(f"Finished in {t2-t1} seconds")

