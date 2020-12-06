function ddxf=derivate_dxf(w, dw, dxdes, ddxdes, A, dA)

ddxf=dw * dxdes + w * ddxdes - dA * dxdes - A * ddxdes;