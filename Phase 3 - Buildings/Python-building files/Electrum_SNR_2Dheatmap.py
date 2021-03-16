import matplotlib.pylab as plt
import numpy as np
from matplotlib import cm as cm

'''
Based on the example taken from matplotlib:
    https://matplotlib.org/gallery/images_contours_and_fields/pcolor_demo.html

Author: Gerard Giramé Rizzo
Last edit: 01/12/2020
'''


def listToMatrix(lst, rows, cols):  # 24, 25
    z_matrix = np.zeros((rows, cols))
    for i in range(0, rows):
        z_matrix[i] = lst[i*cols:(i+1)*cols]
        print(z_matrix[i])
    return z_matrix

# Z = 27 m
#listZ = [0, 0, 11.9973, 12.6552, 13.3045, 13.9335, 14.5346, 15.0955, 15.5844, 16.0371, 16.4066, 16.6756, 32.7611, 32.7959, 32.7612, 16.6758, 16.4069, 16.0454, 15.6027, 15.0953, 14.5343, 13.9306, 13.3044, 12.637, 11.9978, 0, 11.9675, 12.6828, 13.4119, 14.1293, 14.8309, 15.5046, 16.1223, 16.7305, 17.2435, 17.6666, 17.9828, 33.5797, 33.6205, 33.5798, 17.9837, 17.6664, 17.2434, 16.7297, 16.1469, 15.5083, 14.831, 14.1293, 13.4103, 12.6882, 0, 12.5883, 13.3753, 14.1674, 14.9608, 15.7465, 16.5128, 17.2426, 17.9162, 18.5185, 19.0205, 19.3987, 34.4689, 34.5183, 34.469, 19.3988, 19.0203, 18.5195, 17.9192, 17.2432, 16.513, 15.7474, 14.9611, 14.1682, 13.354, 12.3651, 13.1984, 14.0504, 14.9188, 15.7967, 16.6758, 17.543, 18.382, 19.1691, 19.8754, 20.4783, 20.9366, 35.4413, 35.5015, 35.4413, 20.9364, 20.4783, 19.8784, 19.169, 18.3793, 17.5432, 16.6682, 15.7957, 14.9178, 14.0509, 12.8866, 13.7811, 14.7029, 15.633, 16.6212, 17.6035, 18.5891, 19.5526, 20.4781, 21.3239, 22.0512, 22.6157, 36.5104, 36.5866, 36.511, 22.6151, 22.0511, 21.324, 20.4782, 19.5552, 18.589, 17.6043, 16.6211, 15.6514, 14.7011, 13.3757, 14.3259, 15.3222, 16.3532, 17.4218, 18.5192, 19.6347, 20.7497, 21.8363, 22.8529, 23.7467, 24.4544, 37.6953, 37.7926, 37.6955, 24.4547, 23.7469, 22.8534, 21.8361, 20.7501, 19.635, 18.5194, 17.4209, 16.354, 15.3181, 13.8194, 14.8314, 15.895, 17.0092, 18.1802, 19.3987, 20.6585, 21.9431, 23.2236, 24.4547, 25.5671, 26.4712, 39.014, 39.1419, 39.0141, 26.4711, 25.567, 24.4548, 23.2236, 21.9431, 20.6575, 19.3989, 18.1804, 17.0114, 15.8748, 14.2083, 15.2765, 16.4057, 17.6047, 18.8735, 20.2155, 21.6272, 23.0983, 24.6042, 26.0965, 27.4939, 28.6702, 40.4841, 40.6595, 40.4841, 28.6709, 27.4936, 26.0968, 24.6039, 23.0985, 21.6273, 20.215, 18.8739, 17.6034, 16.4066, 14.535, 15.6494, 16.8387, 18.1142, 19.4761, 20.9367, 22.4992, 24.1639, 25.916, 27.7156, 29.4721, 31.025, 42.1086, 42.3568, 42.1087, 31.0248, 29.4726, 27.7149, 25.9161, 24.1637, 22.4995, 20.9362, 19.4768, 18.1143, 16.8416, 14.7875, 15.945, 17.1836, 18.5165, 19.9518, 21.5248, 23.2236, 25.0708, 27.0685, 29.1963, 31.3762, 33.4165, 43.8383, 44.1989, 43.8379, 33.4167, 31.3761, 29.196, 27.0686, 25.0709, 23.2238, 21.5248, 19.9612, 18.5192, 17.1675, 14.9627, 16.1279, 17.4218, 18.8015, 20.3022, 21.9429, 23.7471, 25.7397, 27.9437, 30.3666, 32.9637, 35.5471, 45.4798, 45.997, 45.4801, 44.1983, 42.6186, 41.0307, 39.5491, 38.2006, 36.9828, 35.8796, 34.8769, 33.9589, 17.4135, 15.0424, 16.2494, 17.5432, 18.9469, 20.478, 22.1608, 24.0226, 26.0969, 28.4203, 31.0249, 33.8975, 36.8806, 46.5764, 47.2363, 46.5763, 45.0139, 43.1891, 41.4332, 39.8409, 38.4198, 37.1515, 36.0129, 34.9846, 34.0482, 17.5432, 15.051, 16.249, 17.5411, 18.9465, 20.4771, 22.1607, 24.0226, 26.0963, 28.4208, 31.0243, 33.8974, 36.8811, 46.5766, 47.2363, 46.5763, 45.0139, 43.1897, 41.432, 39.841, 38.42, 37.1515, 36.0127, 34.9847, 34.0484, 17.542, 14.949, 16.1461, 17.422, 18.8016, 20.3022, 21.9431, 23.7472, 25.7396, 27.9436, 30.3665, 32.9638, 35.5471, 45.4799, 45.9965, 37.6438, 35.5482, 32.9632, 30.3667, 27.9437, 25.7395, 23.7471, 21.9431, 20.3021, 18.8015, 17.4219, 14.7884, 15.9446, 17.1836, 18.518, 19.9614, 21.5248, 23.224, 25.0707, 27.0686, 29.1963, 31.3761, 33.4167, 43.8381, 44.1974, 34.9576, 33.4164, 31.3759, 29.1962, 27.0685, 25.0707, 23.224, 21.5248, 19.9616, 18.5193, 17.1843, 14.5146, 15.6313, 16.8396, 18.114, 19.4766, 20.9363, 22.4995, 24.1639, 25.9162, 27.7156, 29.4729, 31.0245, 42.1078, 42.3568, 32.1295, 31.025, 29.473, 27.7153, 25.916, 24.1636, 22.4991, 20.9368, 19.4765, 18.114, 16.8397, 14.2053, 15.2762, 16.4056, 17.6048, 18.8732, 20.2153, 21.6272, 23.0982, 24.6042, 26.0967, 27.4926, 28.671, 40.4842, 40.6595, 29.4728, 28.6707, 27.4939, 26.0969, 24.6037, 23.0985, 21.6271, 20.2157, 18.874, 17.6046, 16.4042, 13.7986, 14.8318, 15.8945, 17.0104, 18.1805, 19.3985, 20.6585, 21.9416, 23.224, 24.4544, 25.5666, 26.4712, 39.0138, 39.1422, 27.0681, 26.4712, 25.5669, 24.4546, 23.2239, 21.9423, 20.6584, 19.3989, 18.1751, 17.0109, 15.8947, 13.3758, 14.3098, 15.3219, 16.3531, 17.4196, 18.5187, 19.635, 20.7501, 21.8363, 22.8532, 23.7469, 37.4158, 37.6954, 37.7924, 24.9121, 24.4547, 23.7472, 22.8533, 21.8363, 20.7495, 19.6335, 18.5169, 17.4217, 16.3517, 15.2998, 12.8874, 13.7814, 14.6996, 15.6486, 16.6213, 17.6046, 18.5886, 19.5555, 20.4784, 21.3234, 22.0508, 36.291, 36.5109, 36.5865, 22.9738, 22.6156, 22.0512, 21.3239, 20.4783, 19.5554, 18.5891, 17.6042, 16.6207, 15.6512, 14.7032, 12.3673, 13.1974, 14.0509, 14.9177, 15.7928, 16.6758, 17.5432, 18.3819, 19.1697, 19.873, 20.4784, 35.2648, 35.4412, 35.5015, 21.2253, 20.9366, 20.4782, 19.8776, 19.1694, 18.3815, 17.5427, 16.6729, 15.7968, 14.9184, 14.0508, 0, 12.5911, 13.3757, 14.1691, 14.9616, 15.748, 16.5129, 17.243, 17.9191, 18.512, 19.0186, 34.3247, 34.4687, 34.5181, 19.6349, 19.3986, 19.0196, 18.5195, 17.9192, 17.2433, 16.5117, 15.7454, 14.9623, 14.1688, 13.3748, 0, 11.9658, 12.6692, 13.4123, 14.1276, 14.8296, 15.4879, 16.1455, 16.7304, 17.2433, 33.2654, 33.4597, 33.5797, 33.6205, 18.18, 17.9836, 17.6644, 17.2408, 16.7067, 16.147, 15.5063, 14.8278, 14.1291, 13.412, 12.6881, 0, 0, 11.9975, 12.6522, 13.3031, 13.933, 14.5339, 15.0955, 15.6032, 16.0421, 32.4954, 32.6599, 32.7607, 32.7958, 16.8414, 16.6717, 16.4065, 16.0413, 15.5838, 15.095, 14.5337, 13.9331, 13.3042, 12.6491, 11.9975]
# Z = 30 m
#listZ = [0, 12.0024, 12.6562, 13.304, 13.9436, 14.5617, 15.1525, 15.7035, 16.1831, 16.6274, 16.9899, 17.2536, 32.706, 32.7404, 32.7061, 17.2538, 16.9902, 16.6357, 16.2016, 15.7033, 15.1522, 14.5588, 13.9433, 13.2857, 12.6566, 0, 12.6268, 13.3352, 14.049, 14.7543, 15.4436, 16.1051, 16.7309, 17.3074, 17.8098, 18.2243, 18.5333, 33.5144, 33.5547, 33.5146, 18.5345, 18.224, 17.8099, 17.3065, 16.7352, 16.1088, 15.4436, 14.7543, 14.048, 13.3364, 12.4774, 13.2381, 14.0129, 14.7916, 15.5712, 16.3424, 17.094, 17.8098, 18.4662, 19.0571, 19.5473, 19.9163, 34.3904, 34.4389, 34.3905, 19.9164, 19.5471, 19.0581, 18.4714, 17.8096, 17.0942, 16.3434, 15.5715, 14.7924, 13.9913, 13.02, 13.839, 14.6767, 15.53, 16.3918, 17.2538, 18.103, 18.9238, 19.6925, 20.3819, 20.9682, 21.414, 35.3452, 35.4042, 35.3452, 21.414, 20.9683, 20.384, 19.6927, 18.9209, 18.1035, 17.2498, 16.3908, 15.5289, 14.6772, 13.5317, 14.412, 15.3179, 16.2308, 17.2003, 18.1622, 19.1261, 20.069, 20.968, 21.7903, 22.4958, 23.0425, 36.3905, 36.4648, 36.3911, 23.0419, 22.4958, 21.7903, 20.968, 20.0692, 19.1259, 18.1634, 17.2002, 16.2492, 15.3161, 14.0135, 14.9473, 15.9261, 16.9375, 17.9845, 19.0575, 20.145, 21.2325, 22.2876, 23.2719, 24.1349, 24.8163, 37.5422, 37.6364, 37.5424, 24.8166, 24.1351, 23.2724, 22.2873, 21.2326, 20.147, 19.058, 17.9836, 16.9384, 15.9219, 14.4498, 15.4441, 16.4882, 17.5804, 18.7265, 19.9163, 21.1435, 22.391, 23.6302, 24.8166, 25.8839, 26.7475, 38.8133, 38.9358, 38.8133, 26.7475, 25.8838, 24.8167, 23.6301, 22.3911, 21.1432, 19.9166, 18.7268, 17.5826, 16.468, 14.8319, 15.8812, 16.9891, 18.1638, 19.404, 20.7124, 22.0846, 23.5091, 24.9603, 26.39, 27.7198, 39.7478, 40.2131, 40.3786, 40.2131, 28.832, 27.7195, 26.3903, 24.96, 23.5093, 22.0848, 20.7119, 19.4042, 18.1622, 16.9899, 15.153, 16.2472, 17.4134, 18.662, 19.9921, 21.4141, 22.9297, 24.5368, 26.2176, 27.9299, 29.5843, 41.1046, 41.7321, 41.961, 41.7321, 31.0298, 29.5848, 27.9292, 26.2177, 24.5367, 22.9301, 21.414, 19.9926, 18.6622, 17.4163, 15.401, 16.5372, 17.7513, 19.055, 20.4634, 21.9854, 23.6302, 25.4083, 27.316, 29.3258, 31.3544, 33.218, 43.3063, 43.6276, 43.3059, 33.2182, 31.3544, 29.3255, 27.3162, 25.4084, 23.6303, 21.9854, 34.5864, 33.7182, 32.9136, 15.5731, 16.7162, 17.9845, 19.3335, 20.7968, 22.3908, 24.1353, 26.0491, 28.1457, 30.4192, 32.8078, 35.1184, 44.7457, 45.1858, 44.746, 43.627, 42.2014, 40.7278, 39.3251, 38.0307, 36.8506, 35.7744, 34.7915, 33.8882, 33.057, 15.6512, 16.8357, 18.1034, 19.4754, 20.9679, 22.6021, 24.4007, 26.3904, 28.5958, 31.0299, 33.6516, 36.2794, 45.6699, 46.209, 45.6698, 44.3435, 42.7214, 41.1046, 39.6032, 38.2422, 37.0146, 35.9047, 34.8972, 33.9762, 33.1301, 15.6598, 16.8359, 18.1026, 19.475, 20.9674, 22.602, 24.4007, 26.3898, 28.5963, 31.0293, 33.6515, 36.2799, 45.6701, 46.209, 45.6698, 44.3435, 42.722, 41.1034, 39.6032, 38.2423, 37.0146, 35.9046, 34.8973, 33.9764, 33.1303, 15.5595, 16.7345, 17.9847, 19.3336, 20.7969, 22.3911, 24.1354, 26.0489, 28.1457, 30.4191, 32.808, 35.1184, 44.7457, 45.1852, 36.9325, 35.1196, 32.8073, 30.4193, 28.1457, 26.0488, 24.1353, 22.3911, 20.7967, 19.3334, 17.9846, 15.4019, 16.5369, 17.7518, 19.0576, 20.465, 21.9854, 23.6305, 25.4082, 27.3161, 29.3258, 31.3544, 33.2182, 43.3061, 43.6261, 34.5979, 33.218, 31.3542, 29.3256, 27.316, 25.4082, 23.6306, 21.9854, 20.4652, 19.0578, 17.7521, 15.1325, 16.2439, 17.4149, 18.6617, 19.9923, 21.4137, 22.93, 24.5369, 26.2177, 27.9299, 29.585, 31.0295, 41.7313, 41.961, 32.0469, 31.0301, 29.5851, 27.9296, 26.2176, 24.5365, 22.9297, 21.4141, 19.9925, 18.6618, 17.4158, 14.8288, 15.881, 16.989, 18.1636, 19.404, 20.7114, 22.0847, 23.509, 24.9603, 26.3902, 27.7185, 28.832, 40.2131, 40.3786, 29.5849, 28.8317, 27.7198, 26.3904, 24.9598, 23.5093, 22.0846, 20.7126, 19.4043, 18.1636, 16.9876, 14.4286, 15.4445, 16.4877, 17.5816, 18.7268, 19.9162, 21.1436, 22.3896, 23.6305, 24.8163, 25.8834, 26.7476, 38.8131, 38.9361, 27.3156, 26.7475, 25.8837, 24.8165, 23.6304, 22.3903, 21.1434, 19.9165, 18.7213, 17.5821, 16.4879, 14.0132, 14.9313, 15.9258, 16.9374, 17.9822, 19.0573, 20.1468, 21.2326, 22.2875, 23.2722, 24.1351, 37.2713, 37.5423, 37.6363, 25.256, 24.8166, 24.1354, 23.2724, 22.2875, 21.232, 20.1467, 19.0553, 17.9843, 16.9372, 15.9022, 13.5346, 14.4124, 15.3145, 16.2464, 17.2003, 18.1635, 19.1255, 20.0694, 20.9683, 21.7897, 22.4955, 36.1764, 36.391, 36.4647, 23.3888, 23.0424, 22.4959, 21.7902, 20.9681, 20.0693, 19.1261, 18.1634, 17.1998, 16.249, 15.3182, 13.0204, 13.8392, 14.6772, 15.5288, 16.3879, 17.2538, 18.1036, 18.9237, 19.6929, 20.3823, 20.9683, 35.1721, 35.3451, 35.4042, 21.6945, 21.4139, 20.9681, 20.3839, 19.6926, 18.9236, 18.1032, 17.2509, 16.3919, 15.5296, 14.6771, 12.478, 13.2409, 14.0136, 14.7934, 15.572, 16.344, 17.0941, 17.8094, 18.4713, 19.0551, 19.5468, 34.2485, 34.3902, 34.4387, 20.1469, 19.9166, 19.547, 19.0581, 18.4714, 17.8098, 17.0934, 16.3414, 15.5727, 14.7931, 14.0121, 0, 12.6251, 13.3213, 14.0495, 14.7524, 15.4423, 16.0885, 16.7339, 17.3073, 17.8098, 33.2042, 33.396, 33.5144, 33.5547, 18.7255, 18.5344, 18.2236, 17.8072, 17.2836, 16.7354, 16.1067, 15.4405, 14.754, 14.0492, 13.3373, 0, 12.0033, 12.6564, 13.3054, 13.9423, 14.5611, 15.1518, 15.7035, 16.2018, 16.6329, 32.4433, 32.606, 32.7056, 32.7404, 17.416, 17.253, 16.9899, 16.6317, 16.1809, 15.703, 15.1516, 14.5612, 13.9428, 13.3034, 12.6563]
# Z = 35 m
#listZ = [12.3267, 12.9637, 13.6021, 14.2326, 14.8557, 15.4566, 16.0308, 16.5654, 17.0296, 17.4602, 17.811, 18.0659, 32.6024, 32.6361, 32.6025, 32.5042, 17.8114, 17.4685, 17.0485, 16.5651, 16.0304, 15.4537, 14.8554, 14.2143, 13.6019, 12.882, 13.5734, 14.265, 14.9582, 15.6439, 16.3132, 16.9545, 17.5645, 18.1179, 18.6032, 19.0026, 19.3, 33.3919, 33.4312, 33.3921, 33.2764, 19.0022, 18.6032, 18.117, 17.5647, 16.9583, 16.3132, 15.6439, 14.9572, 14.2642, 13.4305, 14.1683, 14.9229, 15.6801, 16.437, 17.1845, 17.9116, 18.6033, 19.2402, 19.8041, 20.2744, 20.6281, 34.2436, 34.2907, 34.2437, 34.1056, 20.2742, 19.8046, 19.2405, 18.6029, 17.9118, 17.1854, 16.4383, 15.6808, 14.9013, 13.9574, 14.7538, 15.5685, 16.397, 17.2323, 18.0663, 18.8855, 19.6756, 20.4137, 21.0755, 21.6335, 22.0582, 35.1666, 35.2234, 35.1666, 34.9999, 21.6335, 21.0756, 20.414, 19.674, 18.8863, 18.0652, 17.2318, 16.396, 15.569, 14.4544, 15.3112, 16.1913, 17.0759, 18.0145, 18.9426, 19.8701, 20.7738, 21.6333, 22.4162, 23.0854, 23.6022, 36.1692, 36.2401, 36.1698, 35.9644, 23.0853, 22.4162, 21.6334, 20.7741, 19.8698, 18.9442, 18.0144, 17.0943, 16.1894, 14.9237, 15.8313, 16.7812, 17.761, 18.7714, 19.8046, 20.8486, 21.8856, 22.8881, 23.8187, 24.6302, 25.2678, 37.2623, 37.3511, 37.2625, 37.0064, 24.6304, 23.8191, 22.8878, 21.8855, 20.8489, 19.8046, 18.7708, 17.7614, 16.777, 15.3479, 16.3137, 17.3257, 18.3835, 19.486, 20.6281, 21.8006, 22.9861, 24.1561, 25.2682, 26.2607, 38.1252, 38.4512, 38.5647, 38.4513, 38.1252, 26.2606, 25.2682, 24.156, 22.9862, 21.8007, 20.6284, 19.4862, 18.3839, 17.3055, 15.7194, 16.7377, 17.8102, 18.9443, 20.1371, 21.3894, 22.6956, 24.0422, 25.4023, 26.7286, 27.948, 39.311, 39.7337, 39.8832, 39.7338, 39.3116, 27.9476, 26.7288, 25.4019, 24.0423, 22.6958, 21.3892, 20.1371, 18.9428, 17.811, 16.0311, 17.0932, 18.2115, 19.4238, 20.7006, 22.0583, 23.4957, 25.0066, 26.5694, 28.1392, 29.6301, 40.5326, 41.085, 41.2843, 41.0851, 40.5324, 29.6306, 28.1385, 26.5695, 25.0065, 23.4961, 22.0584, 20.7012, 19.424, 18.2236, 16.2718, 17.3731, 18.5475, 19.8017, 21.1529, 22.6015, 24.156, 25.8194, 27.5793, 29.3991, 31.192, 32.7914, 42.4288, 42.6945, 42.4283, 41.7096, 40.7097, 39.5886, 38.4513, 37.3511, 36.3112, 35.3394, 34.4335, 33.5904, 32.8055, 16.4389, 17.5457, 18.7715, 20.0692, 21.4698, 22.9859, 24.6306, 26.4136, 28.3353, 30.3715, 32.4438, 34.3657, 43.5941, 43.9366, 43.5944, 42.6939, 41.4923, 40.1969, 38.9238, 37.7216, 36.6075, 35.5793, 34.6321, 33.7559, 32.9455, 16.5142, 17.662, 18.8861, 20.2055, 21.6332, 23.1859, 24.8793, 26.7289, 28.7426, 30.9086, 33.156, 43.2747, 44.3048, 44.7042, 44.3047, 43.2754, 41.937, 40.5326, 39.1788, 37.9196, 36.7632, 35.7044, 34.7343, 33.8415, 33.017, 16.523, 17.6624, 18.8858, 20.2049, 21.633, 23.1858, 24.8793, 26.7283, 28.7431, 30.908, 33.1559, 43.2752, 44.305, 44.7042, 44.3047, 43.2753, 41.9376, 40.5314, 39.1789, 37.9197, 36.7632, 35.7042, 34.7344, 33.8417, 33.0172, 16.4252, 17.564, 18.7717, 20.0693, 21.47, 22.9862, 24.6307, 26.4135, 28.3352, 30.3715, 32.4439, 34.3657, 43.5942, 43.936, 43.5945, 42.6945, 41.4917, 40.1969, 38.9238, 37.7223, 36.6078, 35.5798, 34.6318, 33.7566, 32.9454, 16.2727, 17.3728, 18.5473, 19.8047, 21.153, 22.6015, 24.1564, 25.8193, 27.5794, 29.3991, 31.1919, 32.7916, 42.4285, 42.693, 33.9408, 32.7913, 31.1918, 29.3989, 27.5792, 25.8193, 24.1565, 22.6015, 21.1532, 19.8044, 18.5478, 16.0092, 17.0927, 18.2224, 19.4235, 20.7008, 22.0579, 23.496, 25.0067, 26.5695, 28.1392, 29.6309, 30.9082, 41.0842, 41.2843, 31.7921, 30.9087, 29.631, 28.1389, 26.5693, 25.0064, 23.4957, 22.0584, 20.7011, 19.4237, 18.2233, 15.7162, 16.7374, 17.8101, 18.9441, 20.1371, 21.3895, 22.6957, 24.042, 25.4022, 26.7287, 27.9467, 28.9557, 39.7338, 39.8832, 29.6308, 28.9554, 27.948, 26.7289, 25.4017, 24.0423, 22.6956, 21.3896, 20.1371, 18.9442, 17.8087, 15.3267, 16.3141, 17.3251, 18.3827, 19.4863, 20.6276, 21.8007, 22.9846, 24.1564, 25.2679, 26.2603, 27.0579, 38.451, 38.5649, 27.5789, 27.0578, 26.2605, 25.268, 24.1563, 22.9853, 21.8005, 20.6283, 19.4807, 18.3831, 17.3254, 14.9232, 15.8153, 16.7788, 17.7611, 18.769, 19.8039, 20.8487, 21.8855, 22.8879, 23.8189, 24.6303, 37.0065, 37.2624, 37.351, 25.6777, 25.2682, 24.6307, 23.8191, 22.888, 21.8848, 20.8484, 19.8042, 18.7712, 17.7613, 16.7777, 14.4578, 15.3115, 16.1878, 17.0915, 18.0145, 18.944, 19.8698, 20.7747, 21.6335, 22.4155, 23.085, 35.9646, 36.1697, 36.24, 23.9287, 23.6021, 23.0854, 22.416, 21.6335, 20.7746, 19.87, 18.9442, 18.0139, 17.0941, 16.1915, 13.9571, 14.7542, 15.569, 16.3959, 17.2285, 18.0662, 18.8864, 19.6755, 20.414, 21.0757, 21.6336, 34.9998, 35.1665, 35.2234, 22.3252, 22.0581, 21.6334, 21.0755, 20.4137, 19.6757, 18.886, 18.0637, 17.2324, 16.3966, 15.5689, 13.4302, 14.1712, 14.9238, 15.6819, 16.4378, 17.1861, 17.9118, 18.6024, 19.2404, 19.8044, 20.2741, 34.106, 34.2435, 34.2905, 20.8488, 20.6284, 20.2744, 19.8047, 19.2405, 18.6031, 17.9122, 17.1834, 16.4385, 15.6815, 14.9221, 12.8843, 13.5708, 14.2512, 14.9587, 15.642, 16.3119, 16.9518, 17.5638, 18.1177, 18.603, 33.0894, 33.2765, 33.3919, 33.4312, 19.4859, 19.3011, 19.0019, 18.6002, 18.1048, 17.5648, 16.9562, 16.3105, 15.6436, 14.9584, 14.2657, 12.3267, 12.9641, 13.6013, 14.2348, 14.8544, 15.456, 16.03, 16.5653, 17.0484, 17.468, 32.3452, 32.5045, 32.602, 32.636, 18.2231, 18.0664, 17.811, 17.4644, 17.0472, 16.5648, 16.0298, 15.4561, 14.8546, 14.2344, 13.602]
# Z = 40 m
#listZ = [13.1525, 13.7757, 14.398, 15.012, 15.6187, 16.2029, 16.7605, 17.2788, 17.7262, 18.1446, 18.4843, 18.7298, 32.4848, 32.5177, 32.4849, 32.3889, 18.4843, 18.1529, 17.7469, 17.2786, 16.76, 16.2, 15.6184, 14.9938, 14.3977, 13.697, 14.37, 15.0437, 15.7184, 16.3848, 17.0344, 17.6556, 18.2462, 18.7797, 19.248, 19.6323, 19.9183, 33.2535, 33.2917, 33.2537, 33.1412, 19.6321, 19.248, 18.7791, 18.2459, 17.6596, 17.0344, 16.3848, 15.7174, 15.0428, 14.2307, 14.9494, 15.684, 16.4199, 17.1543, 17.8782, 18.581, 19.248, 19.8612, 20.4028, 20.8529, 21.1912, 34.0787, 34.1242, 34.0788, 33.9453, 20.8527, 20.4029, 19.8611, 19.2476, 18.5812, 17.8799, 17.1552, 16.4207, 15.6622, 14.7442, 15.5195, 16.3116, 17.1157, 17.9245, 18.7305, 19.5197, 20.2792, 20.9858, 21.6188, 22.1501, 22.5536, 34.9672, 35.0217, 34.9672, 34.8073, 22.1501, 21.6187, 20.9865, 20.2783, 19.5205, 18.7296, 17.9247, 17.1149, 16.3121, 15.228, 16.0616, 16.9161, 17.7678, 18.6806, 19.5747, 20.4657, 21.3311, 22.1499, 22.8932, 23.5261, 24.0133, 35.9244, 35.9918, 35.925, 35.7296, 23.5261, 22.8932, 22.1499, 21.3311, 20.4652, 19.5762, 18.6804, 17.7909, 16.9142, 15.6848, 16.5667, 17.4879, 18.4358, 19.4099, 20.4029, 21.4019, 22.3898, 23.3398, 24.2168, 24.9774, 36.7159, 36.9565, 37.0398, 36.9568, 36.7162, 24.9776, 24.2173, 23.3395, 22.3897, 21.4021, 20.4027, 19.4101, 18.4363, 17.4837, 16.0973, 17.0349, 18.0148, 19.0366, 20.0969, 21.1912, 22.309, 23.4324, 24.5336, 25.5723, 26.4919, 37.7612, 38.0625, 38.167, 38.0626, 37.7612, 26.4918, 25.5723, 24.5335, 23.4325, 22.3091, 21.1915, 20.0972, 19.0367, 17.9945, 16.4582, 17.4457, 18.483, 19.5762, 20.7214, 21.9176, 23.1577, 24.4268, 25.697, 26.9226, 28.0365, 38.8492, 39.2315, 39.3658, 39.2315, 38.8499, 28.0361, 26.9229, 25.6967, 24.4269, 23.158, 21.9175, 20.7214, 19.576, 18.4838, 16.7607, 17.7909, 18.8808, 20.0372, 21.2605, 22.5538, 23.913, 25.3288, 26.7763, 28.2098, 29.5487, 39.9444, 40.4294, 40.6026, 40.4294, 39.9441, 29.5492, 28.2092, 26.7764, 25.3287, 23.9134, 22.5539, 21.261, 20.0376, 18.8822, 16.9942, 18.0609, 19.1942, 20.4011, 21.6924, 23.0687, 24.5335, 26.084, 27.7011, 29.3429, 30.9236, 32.2968, 41.5782, 41.7995, 41.5777, 40.9692, 40.1006, 39.1006, 38.0626, 37.0398, 36.0592, 35.1328, 34.262, 33.4462, 32.6829, 17.1562, 18.2341, 19.41, 20.6561, 21.9944, 23.4322, 24.9778, 26.6329, 28.3873, 30.2052, 32.0017, 33.6087, 42.5329, 42.8052, 42.5332, 41.799, 40.7824, 39.6464, 38.4965, 37.386, 36.3398, 35.3623, 34.4534, 33.6067, 32.8193, 17.2334, 18.3398, 19.5203, 20.787, 22.1497, 23.621, 25.2101, 26.923, 28.7545, 30.6765, 32.6045, 42.2754, 43.0934, 43.4004, 43.0933, 42.2761, 41.1632, 39.9444, 38.7291, 37.5704, 36.487, 35.4818, 34.5519, 33.6897, 32.8889, 17.2378, 18.3407, 19.5203, 20.7863, 22.1497, 23.621, 25.2101, 26.9224, 28.755, 30.6758, 32.6044, 42.2759, 43.0936, 43.4004, 43.0933, 42.276, 41.1638, 39.9432, 38.7292, 37.5705, 36.487, 35.4816, 34.5519, 33.6898, 32.8891, 17.1426, 18.2452, 19.4102, 20.6565, 21.9945, 23.4325, 24.9779, 26.6328, 28.3872, 30.2052, 32.0018, 33.6088, 42.5329, 42.8047, 42.5332, 41.7995, 40.7819, 39.6463, 38.4965, 37.3867, 36.3401, 35.3629, 34.4531, 33.6074, 32.8192, 16.9952, 18.0609, 19.1941, 20.4029, 21.6923, 23.0687, 24.5339, 26.0839, 27.7012, 29.3429, 30.9236, 32.2971, 41.578, 41.7981, 33.2589, 32.2968, 30.9234, 29.3428, 27.7011, 26.0838, 24.5339, 23.0687, 21.6925, 20.4022, 19.1946, 16.7573, 17.791, 18.881, 20.037, 21.2606, 22.5533, 23.9133, 25.3289, 26.7764, 28.2099, 29.5495, 30.6761, 40.4286, 40.6027, 31.4432, 30.6766, 29.5495, 28.2096, 26.7763, 25.3285, 23.913, 22.5539, 21.261, 20.0371, 18.882, 16.4549, 17.4455, 18.4829, 19.5761, 20.7215, 21.9179, 23.1579, 24.4266, 25.6969, 26.9228, 28.0352, 28.9461, 39.2315, 39.3659, 29.5493, 28.9458, 28.0365, 26.923, 25.6964, 24.4269, 23.1578, 21.9178, 20.7214, 19.5762, 18.4817, 16.074, 17.0352, 18.0143, 19.0362, 20.0972, 21.1912, 22.3091, 23.4309, 24.5338, 25.572, 26.4915, 27.2248, 38.0623, 38.1673, 27.7007, 27.2247, 26.4917, 25.5722, 24.5338, 23.4316, 22.3088, 21.1914, 20.0895, 19.0358, 18.0146, 15.6843, 16.5628, 17.4878, 18.4359, 19.4075, 20.4019, 21.4019, 22.3897, 23.3396, 24.2171, 24.9776, 36.7162, 36.9567, 37.0397, 25.9527, 25.5723, 24.9779, 24.2172, 23.3397, 22.389, 21.402, 20.4011, 19.4098, 18.4362, 17.4875, 15.2315, 16.0619, 16.914, 17.7881, 18.6805, 19.576, 20.4653, 21.3312, 22.1502, 22.8925, 23.5257, 35.7298, 35.9249, 35.9917, 24.3202, 24.0132, 23.5262, 22.8931, 22.1501, 21.3312, 20.4655, 19.5762, 18.6796, 17.7908, 16.9163, 14.7438, 15.5199, 16.3121, 17.1146, 17.9235, 18.7303, 19.5204, 20.2789, 20.9864, 21.6188, 22.1502, 34.8071, 34.9671, 35.0217, 22.807, 22.5536, 22.1499, 21.6188, 20.9862, 20.2792, 19.5202, 18.7301, 17.9247, 17.1152, 16.312, 14.2305, 14.9523, 15.6849, 16.4217, 17.1552, 17.8798, 18.5813, 19.2471, 19.8611, 20.4029, 20.8526, 33.9456, 34.0785, 34.124, 21.402, 21.1915, 20.8529, 20.4029, 19.8611, 19.2479, 18.5818, 17.8771, 17.1558, 16.4214, 15.6831, 13.6981, 14.367, 15.03, 15.7189, 16.3829, 17.033, 17.6589, 18.2454, 18.7798, 19.2475, 32.9593, 33.1414, 33.2535, 33.2917, 20.0971, 19.9194, 19.6323, 19.2449, 18.7789, 18.2463, 17.6572, 17.0334, 16.3845, 15.7186, 15.0444, 13.1526, 13.7751, 14.3968, 15.0144, 15.6175, 16.2023, 16.7596, 17.2788, 17.7468, 18.1528, 32.2337, 32.3893, 32.4844, 32.5176, 18.8819, 18.7307, 18.4829, 18.1507, 17.7468, 17.2782, 16.7594, 16.2024, 15.6176, 15.014, 14.3979]
# Z = 45 m
#listZ = [13.8622, 14.4706, 15.0774, 15.6754, 16.2658, 16.8333, 17.3743, 17.8764, 18.3235, 18.7132, 19.0415, 32.2614, 32.3543, 32.3863, 32.3544, 32.261, 19.0413, 18.7214, 18.3296, 17.8761, 17.3738, 16.8304, 16.2655, 15.6571, 15.077, 14.3941, 15.0501, 15.7063, 16.3626, 17.0099, 17.6405, 18.2406, 18.8117, 19.3267, 19.7767, 20.1462, 32.9917, 33.1006, 33.1376, 33.1008, 32.9919, 20.146, 19.7767, 19.3256, 18.8112, 18.2451, 17.6397, 17.0099, 16.3617, 15.7054, 14.9143, 15.6144, 16.3292, 17.0446, 17.7558, 18.4561, 19.1344, 19.7767, 20.3659, 20.8846, 21.3153, 33.7693, 33.8975, 33.9413, 33.8976, 33.769, 21.3151, 20.8849, 20.3658, 19.7763, 19.1346, 18.458, 17.7577, 17.0447, 16.3083, 15.4147, 16.1693, 16.9389, 17.7185, 18.5008, 19.2786, 20.0379, 20.7665, 21.4427, 22.0458, 22.5507, 34.5966, 34.7499, 34.8019, 34.7499, 34.5969, 22.5507, 22.0457, 21.4428, 20.7658, 20.0387, 19.2777, 18.5012, 17.7183, 16.9393, 15.8856, 16.6961, 17.5251, 18.3654, 19.2305, 20.0918, 20.9451, 21.7718, 22.5505, 23.2546, 35.1835, 35.4762, 35.6601, 35.7239, 35.6607, 35.4756, 23.8516, 23.2546, 22.5505, 21.7717, 20.9447, 20.0921, 19.2303, 18.3718, 17.5232, 16.33, 17.1862, 18.0789, 18.9945, 19.9323, 20.8847, 21.8392, 22.7781, 23.6761, 24.5004, 36.0522, 36.4055, 36.6306, 36.7084, 36.6309, 36.4057, 25.2114, 24.5009, 23.6758, 22.7779, 21.8394, 20.8847, 19.9328, 18.995, 18.0594, 16.7308, 17.6402, 18.588, 19.5733, 20.5918, 21.6382, 22.7014, 23.7634, 24.797, 25.7643, 36.9487, 37.3779, 37.6553, 37.7511, 37.6553, 37.3779, 26.6138, 25.7644, 24.7969, 23.7635, 22.7016, 21.6385, 20.5922, 19.5734, 18.5746, 17.0812, 18.0381, 19.0399, 20.0923, 21.1896, 22.3299, 23.5044, 24.6971, 25.8799, 27.0093, 37.8497, 38.3725, 38.7172, 38.8377, 38.7172, 38.3732, 28.0236, 27.0095, 25.8796, 24.6972, 23.5047, 22.3298, 21.1896, 20.0922, 19.0407, 17.3745, 18.372, 19.4243, 20.5346, 21.7043, 22.9335, 24.2154, 25.5383, 26.8751, 28.1807, 29.3808, 39.3523, 39.7782, 39.929, 39.7782, 39.352, 29.3813, 28.18, 26.8752, 25.5381, 24.2158, 22.9336, 21.7048, 20.535, 19.4247, 17.6008, 18.6328, 19.7253, 20.8846, 22.1159, 23.4204, 24.7969, 26.2378, 27.7197, 29.1977, 30.5909, 40.2454, 40.7642, 40.9502, 40.7638, 40.2456, 39.49, 38.5995, 37.6554, 36.7084, 35.7876, 34.9079, 34.0739, 33.2871, 32.5469, 17.7577, 18.8102, 19.9325, 21.1276, 22.4029, 23.7632, 25.2116, 26.7435, 28.3409, 29.9616, 31.5214, 40.949, 41.5561, 41.777, 41.5564, 40.9497, 40.0847, 39.0881, 38.0522, 37.0305, 36.0522, 35.1266, 34.2578, 33.4422, 32.6795, 17.8346, 18.9018, 20.0385, 21.2522, 22.5503, 23.941, 25.4279, 27.0096, 28.6713, 30.3752, 32.0341, 41.3452, 42.0079, 42.251, 42.0078, 41.3458, 40.4118, 39.3523, 38.2638, 37.2015, 36.1907, 35.2403, 34.3522, 33.5224, 32.7471, 17.8367, 18.9028, 20.0388, 21.2516, 22.5503, 23.9409, 25.4279, 27.009, 28.6718, 30.3745, 32.034, 41.3457, 42.0081, 42.251, 42.0079, 41.3458, 40.4124, 39.3511, 38.2638, 37.2016, 36.1907, 35.2401, 34.3523, 33.5226, 32.7472, 17.7434, 18.8105, 19.9328, 21.1276, 22.4029, 23.7635, 25.2117, 26.7433, 28.3409, 29.9616, 31.5216, 40.949, 41.5562, 41.7764, 41.5565, 40.9502, 40.0841, 39.088, 38.0522, 37.0312, 36.0525, 35.1272, 34.2575, 33.443, 32.6794, 17.6018, 18.6328, 19.7249, 20.885, 22.1156, 23.4204, 24.7973, 26.2378, 27.7198, 29.1977, 30.5908, 40.2457, 40.764, 40.9487, 32.5838, 31.7731, 30.5906, 29.1976, 27.7196, 26.2377, 24.7973, 23.4204, 22.1159, 20.8846, 19.7253, 17.3736, 18.3722, 19.4243, 20.5341, 21.7046, 22.933, 24.2157, 25.5383, 26.8752, 28.1808, 29.3815, 39.3518, 39.7773, 39.929, 31.0415, 30.3753, 29.3816, 28.1805, 26.8751, 25.538, 24.2154, 22.9335, 21.7048, 20.5342, 19.4245, 17.0778, 18.0378, 19.0398, 20.0921, 21.1897, 22.3302, 23.5046, 24.6969, 25.8799, 27.0094, 28.0226, 38.3732, 38.7173, 38.8377, 29.3814, 28.8429, 28.0239, 27.0096, 25.8794, 24.6972, 23.5044, 22.33, 21.1897, 20.0922, 19.0408, 16.727, 17.6405, 18.5869, 19.5732, 20.5922, 21.6384, 22.7016, 23.7619, 24.7972, 25.7641, 26.6134, 37.378, 37.6551, 37.7514, 27.7193, 27.2856, 26.6137, 25.7642, 24.7972, 23.7626, 22.7012, 21.6384, 20.5911, 19.5726, 18.5881, 16.3295, 17.188, 18.0789, 18.9946, 19.9326, 20.884, 21.8393, 22.778, 23.676, 24.5007, 25.2114, 36.4058, 36.6308, 36.7083, 26.1166, 25.7644, 25.2117, 24.5009, 23.6761, 22.7772, 21.8394, 20.8846, 19.9325, 18.9949, 18.0791, 15.8891, 16.6964, 17.5251, 18.3651, 19.2304, 20.0922, 20.9447, 21.7718, 22.5508, 23.2539, 23.8513, 35.4758, 35.6606, 35.7238, 24.5972, 24.3095, 23.8517, 23.2545, 22.5507, 21.7717, 20.9448, 20.0921, 19.2295, 18.3719, 17.5253, 15.4143, 16.1697, 16.9393, 17.718, 18.5003, 19.2784, 20.0388, 20.7662, 21.4428, 22.0458, 22.5508, 34.5967, 34.7497, 34.8019, 23.1731, 22.9333, 22.5505, 22.0458, 21.4426, 20.7666, 20.0383, 19.2782, 18.5013, 17.718, 16.9392, 14.9141, 15.6173, 16.3302, 17.0457, 17.7571, 18.458, 19.135, 19.7758, 20.3658, 20.885, 21.315, 33.7693, 33.8973, 33.9411, 21.8394, 21.6385, 21.3153, 20.885, 20.3658, 19.7766, 19.1353, 18.455, 17.7573, 17.0454, 16.3284, 14.3949, 15.047, 15.6926, 16.3631, 17.008, 17.6383, 18.2448, 18.8109, 19.326, 19.7766, 32.8154, 32.992, 33.1006, 33.1376, 20.5921, 20.4216, 20.1463, 19.7761, 19.3266, 18.8118, 18.2424, 17.6404, 17.0096, 16.3629, 15.7071, 13.8624, 14.4697, 15.0762, 15.6778, 16.2645, 16.8327, 17.3734, 17.8763, 18.3294, 18.7216, 32.1098, 32.2614, 32.3539, 32.3863, 19.4244, 19.2787, 19.041, 18.7199, 18.3293, 17.8758, 17.3732, 16.833, 16.2647, 15.6774, 15.0773]
# Z = 50 m
#listZ = [14.4783, 15.072, 15.6636, 16.2457, 16.8197, 17.3705, 17.8953, 18.3808, 18.8174, 19.1942, 19.5051, 32.1219, 32.212, 32.2431, 32.2122, 32.1215, 31.9747, 19.1967, 18.8188, 18.3805, 17.8944, 17.3676, 16.8194, 16.2263, 15.6632, 14.9974, 15.6369, 16.2767, 16.9137, 17.5418, 18.1529, 18.7355, 19.2839, 19.7799, 20.2122, 20.5668, 32.8295, 32.9347, 32.9704, 32.9349, 32.8296, 32.6591, 20.2123, 19.7788, 19.2835, 18.7373, 18.1518, 17.5417, 16.9128, 16.2748, 15.5046, 16.1863, 16.8812, 17.5764, 18.2641, 18.9407, 19.5946, 20.2124, 20.7773, 21.2738, 21.6847, 33.5789, 33.7021, 33.7441, 33.7022, 33.5787, 33.3802, 21.2739, 20.7772, 20.212, 19.5947, 18.9428, 18.2662, 17.5759, 16.88, 15.992, 16.7259, 17.4729, 18.2282, 18.984, 19.7334, 20.4629, 21.1607, 21.8062, 22.38, 34.1376, 34.3711, 34.5172, 34.5667, 34.5172, 34.3714, 34.1377, 22.3799, 21.8063, 21.1604, 20.4636, 19.7327, 18.9845, 18.2281, 17.4734, 16.4501, 17.2374, 18.041, 18.8582, 19.6871, 20.5138, 21.3313, 22.1196, 22.8587, 23.5242, 34.9293, 35.2064, 35.38, 35.4402, 35.3805, 35.2058, 34.9292, 23.5242, 22.8587, 22.1195, 21.3311, 20.515, 19.687, 18.8594, 18.039, 16.882, 17.7153, 18.5766, 19.4599, 20.3616, 21.2738, 22.1837, 23.0741, 23.9212, 24.6942, 35.7488, 36.0796, 36.2896, 36.362, 36.2899, 36.0799, 35.749, 24.6947, 23.9209, 23.074, 22.1839, 21.2737, 20.3622, 19.4604, 18.5729, 17.2711, 18.1523, 19.068, 20.0171, 20.9935, 21.9924, 23.0016, 24.0033, 24.9711, 25.87, 36.5851, 36.9815, 37.2362, 37.3239, 37.2363, 36.9815, 36.585, 25.8701, 24.9711, 24.0033, 23.0017, 21.9927, 20.9939, 20.0169, 19.0668, 17.611, 18.5369, 19.5035, 20.5151, 21.5648, 22.6496, 23.7596, 24.878, 25.9769, 27.0154, 37.414, 37.8888, 38.1993, 38.3073, 38.1993, 37.8895, 37.4137, 27.0156, 25.9766, 24.878, 23.7599, 22.6496, 21.5648, 20.5151, 19.5045, 17.8951, 18.8598, 19.8739, 20.9388, 22.0553, 23.2211, 24.4275, 25.6606, 26.8926, 28.0799, 29.155, 38.7651, 39.1397, 39.2714, 39.1398, 38.7649, 38.1991, 37.5047, 36.7396, 35.9447, 35.1497, 34.3714, 33.6197, 32.8995, 32.2125, 18.1143, 19.1113, 20.1631, 21.2738, 22.4465, 23.6805, 24.9711, 26.3073, 27.6626, 28.9922, 30.2214, 39.546, 39.9907, 40.1485, 39.9903, 39.5462, 38.8867, 38.0935, 37.2363, 36.3621, 35.5001, 34.6677, 33.8713, 33.1147, 32.3989, 18.2663, 19.2835, 20.3617, 21.5057, 22.7188, 24.0031, 25.3574, 26.772, 28.2244, 29.6693, 31.0278, 40.1472, 40.6555, 40.8378, 40.6558, 40.1479, 39.4069, 38.5307, 37.5985, 36.6609, 35.7489, 34.8753, 34.0474, 33.2642, 32.5274, 18.341, 19.3706, 20.4635, 21.6246, 22.8585, 24.1701, 25.5582, 27.0157, 28.5214, 30.0328, 39.6897, 40.4801, 41.0266, 41.2236, 41.0265, 40.4808, 39.6893, 38.7651, 37.7905, 36.8189, 35.8787, 34.9831, 34.1377, 33.3415, 32.5928, 18.3426, 19.3715, 20.4638, 21.6239, 22.8585, 24.17, 25.5583, 27.0151, 28.5219, 30.0321, 39.6896, 40.4806, 41.0268, 41.2236, 41.0265, 40.4807, 39.6899, 38.7639, 37.7906, 36.819, 35.8787, 34.983, 34.1378, 33.3417, 32.593, 18.255, 19.2826, 20.3622, 21.5056, 22.7189, 24.0033, 25.3575, 26.7719, 28.2243, 29.6693, 31.0279, 40.1473, 40.6556, 40.8372, 40.6559, 40.1484, 39.4063, 38.5307, 37.5985, 36.6616, 35.7492, 34.8759, 34.0471, 33.265, 32.5273, 18.1155, 19.1114, 20.1624, 21.2739, 22.4463, 23.6805, 24.9715, 26.3072, 27.6627, 28.9921, 30.2214, 39.5462, 39.9905, 40.147, 31.932, 31.2436, 30.2212, 28.992, 27.6626, 26.3072, 24.9715, 23.6805, 22.4466, 21.2737, 20.163, 17.8949, 18.8599, 19.8737, 20.9386, 22.0556, 23.2206, 24.4278, 25.6607, 26.8928, 28.08, 29.1558, 38.7646, 39.1389, 39.2714, 30.6134, 30.0329, 29.1559, 28.0797, 26.8926, 25.6604, 24.4274, 23.2211, 22.0558, 20.9388, 19.8737, 17.6075, 18.5369, 19.5034, 20.5145, 21.5649, 22.65, 23.7598, 24.8778, 25.9769, 27.0156, 27.9369, 37.8896, 38.1994, 38.3073, 29.1557, 28.6752, 27.9382, 27.0157, 25.9764, 24.878, 23.7596, 22.6498, 21.5649, 20.515, 19.505, 17.2704, 18.1527, 19.0682, 20.0169, 20.9939, 21.9926, 23.0017, 24.0018, 24.9714, 25.8698, 26.6527, 36.9816, 37.236, 37.3241, 27.6622, 27.2678, 26.6529, 25.8699, 24.9714, 24.0025, 23.0014, 21.9926, 20.9936, 20.0167, 19.0683, 16.8814, 17.716, 18.5768, 19.46, 20.3608, 21.2729, 22.1837, 23.0741, 23.9211, 24.6945, 25.3571, 36.08, 36.2898, 36.3619, 26.1955, 25.8701, 25.3574, 24.6946, 23.9212, 23.0733, 22.1839, 21.2738, 20.3618, 19.4603, 18.5769, 16.4536, 17.2378, 18.0414, 18.8585, 19.687, 20.515, 21.3309, 22.1195, 22.859, 23.5235, 24.0858, 35.206, 35.3805, 35.44, 24.7846, 24.5156, 24.0863, 23.5241, 22.8589, 22.1195, 21.3311, 20.515, 19.6861, 18.8596, 18.0411, 15.9917, 16.7263, 17.4734, 18.2279, 18.9835, 19.7332, 20.4638, 21.1604, 21.8063, 22.38, 22.859, 34.3712, 34.5171, 34.5668, 23.4473, 23.2208, 22.8587, 22.38, 21.806, 21.1608, 20.4633, 19.7334, 18.9845, 18.2276, 17.4733, 15.5044, 16.1892, 16.8823, 17.5766, 18.2657, 18.9427, 19.5956, 20.2121, 20.7772, 21.2739, 21.6844, 33.579, 33.7019, 33.7439, 22.1839, 21.9927, 21.6847, 21.2739, 20.7772, 20.2122, 19.5956, 18.9396, 18.2658, 17.5762, 16.8807, 14.9982, 15.6339, 16.2621, 16.9142, 17.5399, 18.1504, 18.737, 19.283, 19.779, 20.2122, 32.659, 32.8297, 32.9347, 32.9704, 20.9938, 20.8307, 20.5669, 20.2092, 19.7799, 19.2839, 18.7344, 18.1529, 17.5417, 16.914, 16.2765, 14.4785, 15.0711, 15.6623, 16.2481, 16.8184, 17.37, 17.894, 18.3807, 18.8187, 19.197, 31.9745, 32.1218, 32.2117, 32.2431, 19.8739, 19.7336, 19.505, 19.1967, 18.8188, 18.3802, 17.8948, 17.3709, 16.8186, 16.2477, 15.6635]

Z = listToMatrix(listZ, 24, 25)
print(np.shape(Z), np.min(Z), np.max(Z))

fig, ax = plt.subplots()

# Smoother version of the 2d-heatmap
im = plt.imshow(Z.T, cmap=cm.jet,  origin='lower',
                extent=(-115, 115, -130, 115),
                interpolation='bilinear', vmin= 0, vmax=47.2363)

cbar = plt.colorbar(im)
cbar.set_label('SNR [dB]',size=10)
plt.title('2D - Heatmap; Z = 30 m')
plt.grid(linestyle='--', linewidth=0.25)
plt.show()