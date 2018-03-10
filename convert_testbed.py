import ads1256       # import this lib

for i in range(0, 50):
	dac = ads1256.convert(5.0, float(i/10.0))
	print(str(dac))
