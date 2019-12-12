import os

bsn_dir = os.path.dirname(__file__)
r_file_path = "configurations/system_manager/enactor.launch"
abs_file_path = os.path.join(bsn_dir,r_file_path)

f = open(abs_file_path, "r")

data = f.readlines()

f.close()

finished_search = False
Kp_index = -1
Ki_index = -1
Kd_index = -1

for index,line in enumerate(data):
	l = line.strip().split(' ')
	for elem in l:
		if elem[0:4] == "name":
			if elem[6:-1] == "Kp":
				Kp_index = index
			elif elem[6:-1] == "Ki":
				Ki_index = index
			elif elem[6:-1] == "Kd":
				Kd_index = index
				finished_search = True

		if elem[0:5] == "value":
			if index == Kp_index:
				Kp = float(elem[7:-1])
			elif index == Ki_index:
				Ki = float(elem[7:-1])
			elif index == Kd_index:
				Kd = float(elem[7:-1])

	if finished_search == True:
		break

old_Ki = ""
old_Kp = ""
old_Kd = ""

i = data[Kp_index].find('"', data[Kp_index].find("v"), len(data[Kp_index])-1)
i = i+1
while data[Kp_index][i] != '"':
	old_Kp += data[Kp_index][i]
	i += 1

i = data[Ki_index].find('"', data[Ki_index].find("v"), len(data[Ki_index])-1)
i = i+1
while data[Ki_index][i] != '"':
	old_Ki += data[Ki_index][i]
	i += 1

i = data[Kd_index].find('"', data[Kd_index].find("v"), len(data[Kd_index])-1)
i = i+1
while data[Kd_index][i] != '"':
	old_Kd += data[Kd_index][i]
	i += 1

if int(Kd) == 10 and int(Kp) == 100:
	if Ki == 1:
		Ki += 1
	else:
		Ki += 2
	Ki = round(Ki,2)
	Kd = 1
	Kp = 1

	str_Kd = str(Kd)
	str_Ki = str(Ki)
	str_Kp = str(Kp)

	if len(str(Ki)) > 6:
		str_Ki = str(Ki)[0:6]
	
	data[Kd_index] = data[Kd_index].replace(old_Kd, str_Kd)
	data[Ki_index] = data[Ki_index].replace(old_Ki, str_Ki)
	data[Kp_index] = data[Kp_index].replace(old_Kp, str_Kp)
else:
	if Kp == 1:
		Kp += 4
		str_Kp = str(Kp)

		if len(str(Kp)) > 6:
			str_Kp = str(Kp)[0:6]

		data[Kp_index] = data[Kp_index].replace(old_Kp, str_Kp)
	else:
		if Kp < 100:
			Kp += 5
			str_Kp = str(Kp)

			if len(str(Kp)) > 6:
				str_Kp = str(Kp)[0:6]

			data[Kp_index] = data[Kp_index].replace(old_Kp, str_Kp)
		else:
			if Kd == 1:
				Kd += 1
			else:
				Kd += 2
			Kp = 1
			
			str_Kp = str(Kp)
			str_Kd = str(Kd)

			if len(str(Kd)) > 6:
				str_Kd = str(Kd)[0:6]
	
			data[Kp_index] = data[Kp_index].replace(old_Kp, str_Kp)
			data[Kd_index] = data[Kd_index].replace(old_Kd, str_Kd)

fp = open(abs_file_path, "w")

fp.writelines(data)

fp.close()

#Engine update code
'''r_file_path = "configurations/system_manager/engine.launch"
abs_file_path = os.path.join(bsn_dir,r_file_path)

f = open(abs_file_path, "r")

data = f.readlines()

f.close()

finished_search = False
off_index = -1
gain_index = -1

for index,line in enumerate(data):
	l = line.strip().split(' ')
	for elem in l:
		if elem[0:4] == "name":
			if elem[6:-1] == "offset":
				off_index = index
			elif elem[6:-1] == "gain":
				gain_index = index
				finished_search = True

		if elem[0:5] == "value":
			if index == off_index:
				offset = float(elem[7:-1])
			elif index == gain_index:
				gain = float(elem[7:-1])

	if finished_search == True:
		break

old_gain = ""
old_offset = ""

i = data[gain_index].find('"', data[gain_index].find("v"), len(data[gain_index])-1)
i = i+1
while data[gain_index][i] != '"':
	old_gain += data[gain_index][i]
	i += 1

if int(gain) == 5:
	if offset == 0.01:
		offset += 0.04
	else:
		offset += 0.05
	offset = round(offset,2)
	gain = 0.0005

	str_off = str(offset)
	str_gain = str(gain)

	if len(str(gain)) > 6:
		str_gain = str(gain)[0:6]

	i = data[off_index].find('"', data[off_index].find("v"), len(data[off_index])-1)
	i += 1
	while data[off_index][i] != '"':
		old_offset += data[off_index][i]
		i += 1
	
	data[off_index] = data[off_index].replace(old_offset, str_off)
	data[gain_index] = data[gain_index].replace(old_gain, str_gain)
else:
	if gain < 0.001:
		gain += 0.0005
	elif gain < 0.01 and gain >= 0.001:
		if gain == 0.001:
			gain += 0.001
		else:
			gain += 0.002
	elif gain < 0.1 and gain >= 0.01:
		if gain == 0.01:
			gain += 0.01
		else:
			gain += 0.02
	elif gain >= 0.1 and gain < 0.5:
		gain += 0.1
	else:
		if gain < 2:
			gain += 0.5
		else:
			gain = 5

	str_gain = str(gain)

	if len(str(gain)) > 6:
		str_gain = str(gain)[0:6]

	data[gain_index] = data[gain_index].replace(old_gain, str_gain)

fp = open(abs_file_path, "w")

fp.writelines(data)

fp.close()'''