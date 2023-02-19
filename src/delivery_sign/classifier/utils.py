def str2idx(string : str) -> int:
  conversion = {'A1':0,'A2':1,'A3':2,'B1':3,'B2':4,'B3':5,'blank':6}
  if string in conversion:
    return conversion[string]
  else:
    return 6

def idx2str(idx : int) -> str:
  inv_conversion = {0:'A1',1:'A2',2:'A3',3:'B1',4:'B2',5:'B3',6:'blank'}
  return inv_conversion[idx]

def str2onehot(string : str) -> list:
  ret = [0]*7
  ret[str2idx(string)] = 1
  return ret