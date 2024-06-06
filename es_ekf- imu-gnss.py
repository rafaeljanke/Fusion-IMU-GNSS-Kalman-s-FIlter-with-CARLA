#!/usr/bin/env python
# coding: utf-8

# In[1]:


import csv
import pickle
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
from rotations import angle_normalize, rpy_jacobian_axis_angle, skew_symmetric, Quaternion


# In[2]:


#### 1. Data ###################################################################################
with open('data/pt1_data.pkl', 'rb') as file:
    data = pickle.load(file)


# In[3]:


#### 1. Data ###################################################################################
with open('data/data_LASI.csv', newline='') as file:
    #data_LASI = csv.DictReader(file)
    data_LASI = csv.reader(file)
    header = []
    header = next(data_LASI)
    
    rows = []
    for row in data_LASI:
        #print(row)
        rows.append(row)
        
print(header)


# In[4]:


class GroundTruth():
    def __init__(self, p = np.array([None]), v = np.array(None),
                r = np.array(None)):
        self.p = p
        self.v = v
        self.r = r


# In[5]:


class Data():
    def __init__(self, t = np.array(None), data = np.array([None])):
        self.data = data
        self.t = t


# In[6]:


t = np.empty((0,1), float)
p = np.empty((0,3), float)
v = np.empty((0,3), float)
r = np.empty((0,3), float)

imu_f = np.empty((0,3), float)
imu_w = np.empty((0,3), float)

gnss = np.empty((0,3), float)

for i in range(14, len(rows)):
    t = np.append(t, float(rows[i][0]))
    p = np.append(p, [[float(rows[i][9]), float(rows[i][10]), float(rows[i][11])]], axis = 0)
    v = np.append(v, [[0, 0, 0]], axis = 0)
    
    if float(rows[i][14]) > 170:
        r = np.append(r, [[float(rows[i][13]), float(rows[i][12]), (-1)*float(rows[i][14])]], axis = 0)

    else:
        r = np.append(r, [[float(rows[i][13]), float(rows[i][12]), float(rows[i][14])]], axis = 0)
        
    imu_f = np.append(imu_f, [[float(rows[i][3]), float(rows[i][4]), float(rows[i][5])]], axis = 0)
    imu_w = np.append(imu_w, [[float(rows[i][6]), float(rows[i][7]), float(rows[i][8])]], axis = 0)
    gnss = np.append(gnss, [[float(rows[i][1]), float(rows[i][2]), 0]], axis = 0)

r = angle_normalize(r)
#imu_w = angle_normalize(imu_w)


# In[7]:


gt = GroundTruth(p, v, r)


# In[8]:


imu_f = Data(t, imu_f)
imu_w = Data(t, imu_w)
gnss = Data(t, gnss)


# In[9]:


gnss.data.shape


# In[10]:


a = 6378137
b = 6356752.3142
f = (a - b) / a
e_sq = f * (2-f)

def geodetic_to_ecef(lat, lon, h):
    # (lat, lon) in WSG-84 degrees
    # h in meters
    lamb = math.radians(lat)
    phi = math.radians(lon)
    s = math.sin(lamb)
    N = a / math.sqrt(1 - e_sq * s * s)

    sin_lambda = math.sin(lamb)
    cos_lambda = math.cos(lamb)
    sin_phi = math.sin(phi)
    cos_phi = math.cos(phi)

    x = (h + N) * cos_lambda * cos_phi
    y = (h + N) * cos_lambda * sin_phi
    z = (h + (1 - e_sq) * N) * sin_lambda

    return x, y, z

def ecef_to_enu(x, y, z, lat0, lon0, h0):
    lamb = math.radians(lat0)
    phi = math.radians(lon0)
    s = math.sin(lamb)
    N = a / math.sqrt(1 - e_sq * s * s)

    sin_lambda = math.sin(lamb)
    cos_lambda = math.cos(lamb)
    sin_phi = math.sin(phi)
    cos_phi = math.cos(phi)

    x0 = (h0 + N) * cos_lambda * cos_phi
    y0 = (h0 + N) * cos_lambda * sin_phi
    z0 = (h0 + (1 - e_sq) * N) * sin_lambda

    xd = x - x0
    yd = y - y0
    zd = z - z0

    xEast = -sin_phi * xd + cos_phi * yd
    yNorth = -cos_phi * sin_lambda * xd - sin_lambda * sin_phi * yd + cos_lambda * zd
    zUp = cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd + sin_lambda * zd

    return xEast, yNorth, zUp

def geodetic_to_enu(lat, lon, h, lat_ref, lon_ref, h_ref):
    x, y, z = geodetic_to_ecef(lat, lon, h)
    
    return ecef_to_enu(x, y, z, lat_ref, lon_ref, h_ref)




    
point_latitude=48.99950150520519
point_longitude=8.002271254544008
point_altitude=1.9948198795318604


origin_latitude=49.000000
origin_longitude=8.00000
origin_altitude=0.0000




# In[11]:


gnss.data[0]


# In[12]:


#x,y,z=geodetic_to_ecef(point_latitude,point_longitude,point_altitude)
latitude = []
for i in range(0,gnss.data.shape[0]):
    point_latitude = origin_latitude + abs(gnss.data[i][1])
    point_longitude = origin_longitude + abs(gnss.data[i][1])
    point_altitude  = origin_altitude
    x,y,z=geodetic_to_ecef(point_latitude,point_longitude,point_altitude)
    carla_x,carla_y,carla_z=ecef_to_enu(x,y,z,origin_latitude,origin_longitude,origin_altitude)

    print ("The converted values from Geodetic to  carla are",carla_x,-carla_y,carla_z)


# In[13]:


#x,y,z=geodetic_to_ecef(point_latitude,point_longitude,point_altitude)
latitude = []
for i in range(0,gnss.data.shape[0]):
    point_latitude = 49+gnss.data[i][1]
    point_longitude = 8+gnss.data[i][1]
    point_altitude  = origin_altitude
    x,y,z=geodetic_to_ecef(point_latitude,point_longitude,point_altitude)
    carla_x,carla_y,carla_z=ecef_to_enu(x,y,z,origin_latitude,origin_longitude,origin_altitude)

    print ("The converted values from Geodetic to  carla are",carla_x,-carla_y,carla_z)
    latitude.append(carla_y)


# In[14]:


longitude = []

for i in range(0,gnss.data.shape[0]):
    point_latitude = 49+gnss.data[i][0]
    point_longitude = 8+gnss.data[i][1]
    point_altitude  = origin_altitude
    x,y,z=geodetic_to_ecef(point_latitude,point_longitude,point_altitude)
    carla_x,carla_y,carla_z=ecef_to_enu(x,y,z,origin_latitude,origin_longitude,origin_altitude)

    print ("The converted values from Geodetic to  carla are",carla_x,-carla_y,carla_z)
    #gnss.data[i][1] = -carla_y
    longitude.append(-carla_y)


# In[15]:


latitude


# In[16]:


longitude


# In[17]:


for i in range(0, gnss.data.shape[0]):
    gnss.data[i][0] = latitude[i]
    gnss.data[i][1] = longitude[i]


# In[18]:


gnss.data


# In[19]:


for i in range(0, imu_f.data.shape[0]):
    imu_f.data[i][2] = imu_f.data[i][2] + 9.81


# In[20]:


imu_f.data


# In[21]:


################################################################################################
#Cada elemento do dicionário de dados é armazenado como um item do dicionário de dados,
#que iremos armazenar em variáveis locais, descritas a seguir:
#   gt: Objeto de dados contendo o Ground Truth, com os seguintes campos:
#     a: Aceleração do veículo, no referencial inercial
#     v: Velocidade do veículo, no referencial inercial
#     p: Posição do veículo, no referencial inercial
#     alpha: Aceleração rotacional do veículo, no referencial inercial
#     w: Velocidade rotacional do veículo, no referencial inercial
#     r: Posição rotacional do veículo, em ângulos de Euler (XYZ) no referencial inercial
#     _t: Timestamp in ms.
#   imu_f: Objeto StampedData com os dados de força específica do IMU (dados no referencial do veículo).
#     data: Os dados reais
#     t: Timestamps in ms.
#   imu_w: Objeto StampedData com a velocidade rotacional do IMU (dados no referencial do veículo).
#     data: Os dados reais
#     t: Carimbos de data/hora em ms.
#   gnss: Objeto StampedData com os dados GNSS..
#     data: Os dados reais
#     t: Timestamps in ms.
#   lidar: Objeto StampedData com os dados do LIDAR (somente posições).
#     data: Os dados reais
#     t: Timestamps in ms.
################################################################################################

#gt = data['gt']
#imu_f = data['imu_f']
#imu_w = data['imu_w']
#gnss = data['gnss']
#lidar = data['lidar']


# In[22]:


gt.p


# In[23]:


for row in gt.r:
    print(row)


# In[24]:


for i in range(0, gt.r.shape[0]):
    if gt.r[i][2] > 170:
        print(gt.r[i][2])


# In[25]:


################################################################################################
# Plot da visão verdadeira do chão para comparação posterior com o modelo
################################################################################################
gt_fig = plt.figure()
ax = gt_fig.add_subplot(111, projection='3d')
ax.plot(gt.p[:,0], gt.p[:,1], gt.p[:,2])
ax.set_xlabel('x [m]')
ax.set_ylabel('y [m]')
ax.set_zlabel('z [m]')
ax.set_title('Ground Truth trajectory')
ax.set_zlim(-1, 5)
plt.show()


# In[26]:


gt_fig = plt.figure()
bx = gt_fig.add_subplot(111)
bx.plot(gt.p[:,0], gt.p[:,1])
bx.set_xlabel('x [m]')
bx.set_ylabel('y [m]')
bx.set_title('Ground Truth trajectory XY')
#ax.set_zlim(-1, 5)
plt.show()


# In[27]:


gt_fig = plt.figure()
ax = gt_fig.add_subplot(111)
ax.plot(gt.p[:,0], gt.p[:,2])
ax.set_xlabel('x [m]')
#ax.set_ylabel('y [m]')
ax.set_ylabel('z [m]')
ax.set_title('Ground Truth trajectory')
plt.show()


# In[28]:


gt_fig = plt.figure()
ax = gt_fig.add_subplot(111)
ax.plot(gt.p[:,1], gt.p[:,2])
ax.set_xlabel('y [m]')
#ax.set_ylabel('y [m]')
ax.set_ylabel('z [m]')
ax.set_title('Ground Truth trajectory')
plt.show()


# In[29]:


################################################################################################
#Lembre-se de que nossos dados de LIDAR são, na verdade, apenas um conjunto de posições
#estimadas a partir de um sistema de correspondência de varredura separado, então podemos
#inseri-los em nosso solucionador como outra medida de posição, assim como fazemos para o GNSS.
#No entanto, o referencial do LIDAR não é o mesmo que o referencial compartilhado
#pelo IMU e pelo GNSS. Para resolver isso, transformamos os dados do LIDAR para
#o referencial do IMU usando nossa matriz de rotação de calibração extrínseca conhecida
#C_li e o vetor de translação t_i_li.
################################################################################################
# Correct calibration rotation matrix, corresponding to Euler RPY angles (0.05, 0.05, 0.1).
#C_li = np.array([
#   [ 0.99376, -0.09722,  0.05466],
#   [ 0.09971,  0.99401, -0.04475],
#   [-0.04998,  0.04992,  0.9975 ]
#])

# Incorrect calibration rotation matrix, corresponding to Euler RPY angles (0.05, 0.05, 0.05).
# C_li = np.array([
#      [ 0.9975 , -0.04742,  0.05235],
#      [ 0.04992,  0.99763, -0.04742],
#      [-0.04998,  0.04992,  0.9975 ]
# ])

#t_i_li = np.array([0.5, 0.1, 0.5])

# Transforma do referencial do LIDAR para o referencial do veículo (IMU).
#lidar.data = (C_li @ lidar.data.T).T + t_i_li


# In[30]:


#### 2. Constantes ##############################################################################
################################################################################################
#Agora que nossos dados estão configurados, podemos começar a preparar as coisas 
#para o algoritmo. Um dos aspectos mais importantes de um filtro 
#é definir corretamente as variâncias estimadas dos sensores. Definimos os valores aqui.
################################################################################################
"""
var_imu_f = 0.10
var_imu_w = 0.25
var_gnss  = 0.01
var_lidar = 1.00
"""

var_imu_f = 0.1
var_imu_w = 0.25
var_gnss = 0.01

################################################################################################
# Também podemos configurar algumas constantes que não mudarão em nenhuma iteração para o algoritmo.
################################################################################################
g = np.array([0, 0, -9.81])  # Gravidade
l_jac = np.zeros([9, 6])
l_jac[3:, :] = np.eye(6)  # Jacobiano modelo de movimento ruído
h_jac = np.zeros([3, 9])
h_jac[:, :3] = np.eye(3)  # Jacobiano modelo de medida


# In[31]:


#### 3. Valores Iniciais #########################################################################
################################################################################################
# Vamos configurar alguns valores iniciais para o nosso solucionador ES-EKF.
################################################################################################
p_est = np.zeros([imu_f.data.shape[0], 3])  # Posição Estimada
v_est = np.zeros([imu_f.data.shape[0], 3])  # Velocidade Estimada
q_est = np.zeros([imu_f.data.shape[0], 4])  # Orientação dos Quaternions Estimadas
p_cov = np.zeros([imu_f.data.shape[0], 9, 9])  # Matriz de Covariância para cada passo de tempo

# Inicializando valores.
p_est[0] = gt.p[0]
v_est[0] = gt.v[0]
q_est[0] = Quaternion(euler=gt.r[0]).to_numpy()
p_cov[0] = np.zeros(9)  # covariance of estimate
gnss_i  = 0
lidar_i = 0
gnss_t = list(gnss.t)


# In[32]:


#gnss.t[1:len(gnss_t):10].shape


# In[33]:


gnss_t = list(gnss.t[1:len(gnss_t):20])


# In[34]:


gt_fig = plt.figure()
bx = gt_fig.add_subplot(111)
bx.plot(gt.p[:,0], gt.p[:,1])
bx.scatter(gnss.data[::20,0], gnss.data[::20,1], marker = 'o', color = 'red')
bx.set_xlabel('x [m]')
bx.set_ylabel('y [m]')
bx.set_title('Ground Truth trajectory XY')
bx.legend('')
#ax.set_zlim(-1, 5)
plt.show()


# In[35]:


print(gt.p[0], gt.v[0], q_est[0])


# In[36]:


#### 4. Atualização das medidas #####################################################################
################################################################################################
#Como precisaremos de uma atualização de medição tanto para os dados do GNSS
#quanto para os do LIDAR,vamos criar uma função para isso.
################################################################################################
def measurement_update(sensor_var, p_cov_check, y_k, p_check, v_check, q_check):
    # 1 Calcula o ganho de Kalman
    r_cov = np.eye(3)*sensor_var
    k_gain = p_cov_check @ h_jac.T @ np.linalg.inv((h_jac @ p_cov_check @ h_jac.T) + r_cov)

    # 2 Computa o erro do estadoCompute error state
    error_state = k_gain @ (y_k - p_check)

    # 3 Calcula Estado corretamente predito
    p_hat = p_check + error_state[0:3]
    v_hat = v_check + error_state[3:6]
    q_hat = Quaternion(axis_angle=error_state[6:9]).quat_mult_left(Quaternion(*q_check))

    # 4 Calcula a covariância corrigida
    p_cov_hat = (np.eye(9) - k_gain @ h_jac) @ p_cov_check


    return p_hat, v_hat, q_hat, p_cov_hat


# In[37]:


for k in range(1, imu_f.data.shape[0]):  # start at 1 b/c we have initial prediction from gt
    delta_t = imu_f.t[k] - imu_f.t[k - 1]

    # 1. Atualizar estado com entradas do IMU
    q_prev = Quaternion(*q_est[k - 1, :]) # orientação anterior como um objeto quaternion
    q_curr = Quaternion(axis_angle=(imu_w.data[k - 1]*delta_t)) # orientação atual do IMU
    c_ns = q_prev.to_mat() # orientação anterior como uma matriz
    f_ns = (c_ns @ imu_f.data[k - 1]) + g # calcular soma das forças
    p_check = p_est[k - 1, :] + delta_t*v_est[k - 1, :] + 0.5*(delta_t**2)*f_ns
    v_check = v_est[k - 1, :] + delta_t*f_ns
    q_check = q_prev.quat_mult_left(q_curr)

    # 1.1 Linearizar o modelo de movimento e calcular Jacobianos
    f_jac = np.eye(9) # jacobiano do modelo de movimento em relação ao último estado
    f_jac[0:3, 3:6] = np.eye(3)*delta_t
    f_jac[3:6, 6:9] = -skew_symmetric(c_ns @ imu_f.data[k - 1])*delta_t

    # 2. Propagar incerteza
    q_cov = np.zeros((6, 6)) # IMU noise covariance
    q_cov[0:3, 0:3] = delta_t**2 * np.eye(3)*var_imu_f
    q_cov[3:6, 3:6] = delta_t**2 * np.eye(3)*var_imu_w
    p_cov_check = f_jac @ p_cov[k - 1, :, :] @ f_jac.T + l_jac @ q_cov @ l_jac.T

    # 3. Verificar disponibilidade das medições do GNSS e do LIDAR
    if imu_f.t[k] in gnss_t:
        #print(k)
        #gnss_i = gnss_t.index(imu_f.t[k])
        #print(gnss_i)
        p_check, v_check, q_check, p_cov_check =             measurement_update(var_gnss, p_cov_check, gnss.data[k], p_check, v_check, q_check)
            #gt.p[gnss_i]
    
#    if imu_f.t[k] in lidar_t:
#        lidar_i = lidar_t.index(imu_f.t[k])
#        p_check, v_check, q_check, p_cov_check = \
#            measurement_update(var_lidar, p_cov_check, lidar.data[lidar_i], p_check, v_check, q_check)

    # Atualizar estados e salvar
    p_est[k, :] = p_check
    v_est[k, :] = v_check
    q_est[k, :] = q_check
    p_cov[k, :, :] = p_cov_check
    
    #print(f'k: {k}\nTempo: {imu_f.t[k]}\nPosição: {p_est[k]}\nVelocidade: {v_est[k]}\nQuaternion: {q_est[k]}\n')


# In[38]:


gnss.data


# In[39]:


gnss_t


# In[40]:


#### 6. Resultados e Análises ###################################################################
################################################################################################
#Agora que temos estimativas de estado para todos os nossos dados de sensor, vamos plotar os resultados.
#Este gráfico mostrará a verdade terrestre e as trajetórias estimadas no mesmo gráfico.
################################################################################################
error_fig, ax = plt.subplots(2, 3)
error_fig.suptitle('Error Plots')
num_gt = gt.p.shape[0]
p_est_euler = []
p_cov_euler_std = []

for i in range(len(q_est)):
    qc = Quaternion(*q_est[i, :])
    p_est_euler.append(qc.to_euler())

    # Aproximação de primeira ordem da covariância RPY
    J = rpy_jacobian_axis_angle(qc.to_axis_angle())
    p_cov_euler_std.append(np.sqrt(np.diagonal(J @ p_cov[i, 6:, 6:] @ J.T)))

p_est_euler = np.array(p_est_euler)
p_cov_euler_std = np.array(p_cov_euler_std)

# Obter estimativas de incerteza da matriz P
p_cov_std = np.sqrt(np.diagonal(p_cov[:, :6, :6], axis1=1, axis2=2))

titles = ['Easting', 'Northing', 'Up', 'Roll', 'Pitch', 'Yaw']
for i in range(3):
    ax[0, i].plot(range(num_gt), gt.p[:, i] - p_est[:num_gt, i])
    ax[0, i].plot(range(num_gt),  3 * p_cov_std[:num_gt, i], 'r--')
    ax[0, i].plot(range(num_gt), -3 * p_cov_std[:num_gt, i], 'r--')
    ax[0, i].set_title(titles[i])
ax[0,0].set_ylabel('Meters')

for i in range(3):
    ax[1, i].plot(range(num_gt),         angle_normalize(gt.r[:, i] - p_est_euler[:num_gt, i]))
    ax[1, i].plot(range(num_gt),  3 * p_cov_euler_std[:num_gt, i], 'r--')
    ax[1, i].plot(range(num_gt), -3 * p_cov_euler_std[:num_gt, i], 'r--')
    ax[1, i].set_title(titles[i+3])
ax[1,0].set_ylabel('Radians')
plt.show()


# In[ ]:


################################################################################################
#Também podemos plotar o erro para cada um dos 6 graus de liberdade, com estimativas para nossa incerteza
#incluídas. As estimativas de erro estão em azul, e os limites de incerteza são vermelhos e tracejados.
#Os limites de incerteza são +/- 3 desvios padrão baseados em nossa incerteza (covariância).
################################################################################################

error_fig, ax = plt.subplots(1, 3, constrained_layout = True)
error_fig.suptitle('Erros estimativa X Ground Truth')
num_gt = gt.p.shape[0]
p_est_euler = []
p_cov_euler_std = []

# Converter quaternions estimados para ângulos de Euler
for i in range(len(q_est)):
    qc = Quaternion(*q_est[i, :])
    p_est_euler.append(qc.to_euler())

    # Aproximação de primeira ordem da covariância RPY
    J = rpy_jacobian_axis_angle(qc.to_axis_angle())
    p_cov_euler_std.append(np.sqrt(np.diagonal(J @ p_cov[i, 6:, 6:] @ J.T)))

p_est_euler = np.array(p_est_euler)
p_cov_euler_std = np.array(p_cov_euler_std)

# Obter estimativas de incerteza da matriz P
p_cov_std = np.sqrt(np.diagonal(p_cov[:, :6, :6], axis1=1, axis2=2))

#titles = ['Easting', 'Northing', 'Up', 'Roll', 'Pitch', 'Yaw']
titles = ['Eixo X', 'Eixo Y', 'Eixo Z']

for i in range(0,3):
    ax[i].plot(range(num_gt), gt.p[:, i] - p_est[:num_gt, i])
    ax[i].plot(range(num_gt),  3 * p_cov_std[:num_gt, i], 'r--')
    ax[i].plot(range(num_gt), -3 * p_cov_std[:num_gt, i], 'r--')
    ax[i].set_title(titles[i])
ax[0].set_ylabel('Metros')

#for i in range(3):
#    ax[1, i].plot(range(num_gt), \
#        angle_normalize(gt.r[:, i] - p_est_euler[:num_gt, i]))
#    ax[1, i].plot(range(num_gt),  3 * p_cov_euler_std[:num_gt, i], 'r--')
#    ax[1, i].plot(range(num_gt), -3 * p_cov_euler_std[:num_gt, i], 'r--')
#    ax[1, i].set_title(titles[i+3])
#ax[1,0].set_ylabel('Radians')

plt.show()


# In[ ]:


error_fig, ax = plt.subplots(1, 3)
error_fig.suptitle('Erros estimativa X Ground Truth  dos ângulos')


titles = ['Roll', 'Pitch', 'Yaw']
for i in range(3):
    ax[i].plot(range(num_gt),         0.1*angle_normalize(gt.r[:, i] - p_est_euler[:num_gt, i]))
    ax[i].plot(range(num_gt),  3 * p_cov_euler_std[:num_gt, i], 'r--')
    ax[i].plot(range(num_gt), -3 * p_cov_euler_std[:num_gt, i], 'r--')
    ax[i].set_title(titles[i])
ax[0].set_ylabel('Radianos')
plt.show()


# In[ ]:


for i in range(0, len(gt.r)):
    print(angle_normalize(gt.r))


# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:




