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

old_gain = gain
old_offset = offset

if gain == 0.5:
	offset += 0.1
	gain = 0.02

	str_offset = str(offset)
	str_gain = str(gain)

	if len(str(offset)) > 4:
		str_off = str(offset)[0:4]
	if len(str(gain)) > 4:
		str_gain = str(gain)[0:4]

	data[off_index] = data[off_index].replace(str(old_offset), str_off)
	data[gain_index] = data[gain_index].replace(str(old_gain), str_gain)
else:
	gain += 0.02

	str_gain = str(gain)

	if len(str(gain)) > 4:
		str_gain = str(gain)[0:4]

	data[gain_index] = data[gain_index].replace(str(old_gain), str_gain)

fp = open(abs_file_path, "w")

fp.writelines(data)

fp.close()

#r_file_path = "configurations/system_manager/enactor.launch"
#abs_file_path = os.path.join(bsn_dir,r_file_path)

#f = open(abs_file_path, "r")

#data = f.readlines()

#f.close()

#Fazer para o enactor mas primeiramente deve-se mudar o código do mesmo para receber o Kp pela launch file