def castType(type,value):
	if type == 0:
		return float(value)	
	if type == 1:
		return int(value)
	if type == 2:
		return bool(int(value))
