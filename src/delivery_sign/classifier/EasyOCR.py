import easyocr
import numpy as np
import torch
import torch.nn as nn
from classifier.utils import str2onehot


class EasyOCR(nn.Module):
    
    def __init__(self, **config):
        super(EasyOCR, self).__init__()
        self.reader = easyocr.Reader(['en'])

        config.setdefault('n_width',64)
        config.setdefault('n_height',64)
        config.setdefault('batch_size',8)
        config.setdefault('allowlist', "123AB")

        self.config = config

    def forward(self, x):
        is_torch = False
        if isinstance(x, torch.Tensor):
            is_torch = True
            x *= 255.0
            x = np.array(x.detach().cpu(), dtype = np.uint8)
            x = x.transpose(0,2,3,1)

        out = self.readbatch(x)
        
        if is_torch:
            out = [str2onehot(z) for z in out]
            out = torch.Tensor(out)
        return out
    
    def readbatch(self, x):
        out = self.reader.readtext_batched(x, detail = 0, **self.config)
        out = [z[0] if len(z)>0 else '' for z in out]
        return out
