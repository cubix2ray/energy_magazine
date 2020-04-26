import scipy.io as spio

battery_data = spio.loadmat('./B0007.mat', squeeze_me=True)['B0007']
print(battery_data['cycle'].item()['data'][0]['Voltage_measured'].item().shape)
print(battery_data['cycle'].item()['data'][0]['Current_measured'].item().shape)
print(battery_data['cycle'].item()['data'][0]['Time'].item())
print(battery_data['cycle'].item()['data'][1]['Voltage_measured'].item().shape)
print(battery_data['cycle'].item()['data'][1]['Current_measured'].item().shape)
print(battery_data['cycle'].item()['data'][1]['Time'].item().shape)
