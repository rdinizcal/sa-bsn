import os

bsn_dir = os.path.dirname(__file__)
r_file_path = "configurations/system_manager/engine.launch"
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

fp.close()