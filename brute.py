import math

# Esta funcion devuelve un candidato a camino minimo en la forma de una lista de aristas ordenadas segun el orden de recorrido
# LCH lista de aristas (cada una es una lista con 2 vertices) que forman el convex hull
# vinicio es una lista que contiene el vertice inicial de los caminos (vinicio[0] es X vinicio[1] es Y)
# vfin es una lista que contiene el vertice final de los caminos (vfin[0] es X vfin[1] es Y)
def extraerCamino(vinicio, vfin, LCH):
	#LC1 es la lista de aristas del camino 1
	LC1 = []
	FIN = False
	A = LCH[0]
	LC1.append(LCH[0])
	del LCH[0]
	#chequeo si el camino tiene solo una arista, o sea si la arista que tome es (vinicio, vfin)
	if (A[0][0] == vfin[0] and A[0][1] == vfin[1]) or (A[1][0] == vfin[0] and A[1][1] == vfin[1]):
		FIN = True
	while not FIN:
		k = 0
		esta = (LCH[k][0][0] == A[1][0] and LCH[k][0][1] == A[1][1]) or (LCH[k][1][0] == A[1][0] and LCH[k][1][1] == A[1][1])
		while not esta:
			k += 1
			esta = (LCH[k][0][0] == A[1][0] and LCH[k][0][1] == A[1][1]) or (LCH[k][1][0] == A[1][0] and LCH[k][1][1] == A[1][1])
		if (LCH[k][0][0] == vfin[0] and LCH[k][0][1] == vfin[1]) or (LCH[k][1][0] == vfin[0] and LCH[k][1][1] == vfin[1]):
			FIN = True
		if (LCH[k][1][0] == A[1][0]) and (LCH[k][1][1] == A[1][1]):
			A__ = [LCH[k][1], LCH[k][0]]
			LC1.append(A__)
			A = A__
		else:
			LC1.append(LCH[k])
			A = LCH[k]
		del LCH[k]
	return LC1

# Recibe una lista de vertices. Cada vertice es una lista con dos valores: X Y
# Devuelve una lista de todas las aristas que forman el convex hull
# Cada arista es una lista con 2 vertices
# Debido al metodo no tienen un orden para extraer los caminos de manera directa
# De eso se encarga la funcion extraerCamino
def fuerzabruta(vertices):
	N = len(vertices)
	#primPunto y segPunto son 2 vertices que dividen al plano, si todos los demas vertices se encuentran en uno solo de los semiplanos entonces estos 2 vertices forman parte de la envoltura
	posPrimPunto = 0
	posSegPunto = 1
	listaEnvolt = []
	
	#genero todas las aristas posibles
	while posPrimPunto < N-1:
		primPunto = vertices[posPrimPunto]
		posSegPunto = posPrimPunto + 1
		while posSegPunto <= N-1:
			segPunto = vertices[posSegPunto]
			cont2 = 0
			cont1 = 0
			n_ = 0
			while (cont1 == 0 or cont2 == 0) and (n_ <= N-1):
				if (n_ != posPrimPunto) and (n_ != posSegPunto):
					d1 = ((vertices[n_][0] - primPunto[0])*(segPunto[1] - primPunto[1]))
					d2 = ((vertices[n_][1] - primPunto[1])*(segPunto[0] - primPunto[0]))
					d = d1 - d2
					if d >= 0:
						cont1 += 1
					else:
						cont2 += 1
				n_ += 1
			if cont1 == 0 or cont2 == 0:
				#listaEnvolt es una lista de aristas (pares de vertices). Cada arista es una lista con dos vertices. Cada vertices es una lista con 2 valores (X e Y)
				arista = [primPunto, segPunto]
				listaEnvolt.append(arista)	
			posSegPunto += 1
		posPrimPunto += 1
		
	return listaEnvolt
	
# Imprime distancias y puntos de cada camino y cual es el mas conveniente
# Ambos caminos son una lista ya ordenada con las aristas que forman cada camino 
def elegirCamino(camino1, camino2):
	k = 0
	acum1 = 0
	while k < len(camino1):
		acum1 += math.sqrt(pow((camino1[k][0][0] - camino1[k][1][0]), 2) + pow((camino1[k][0][1] - camino1[k][1][1]), 2))
		k += 1
	print("Camino 1: Longitud "+ acum1)
	print("Recorrido: ")
	print (camino1[0][0])
	k = 0
	while k < len(camino1):
		print(camino1[k][1])
		k += 1
	
	
	k = 0
	acum2 = 0
	while k < len(camino2):
		acum2 += math.sqrt(pow((camino2[k][0][0] - camino2[k][1][0]), 2) + pow((camino2[k][0][1] - camino2[k][1][1]), 2))
		k += 1
	print("Camino 2: Longitud "+ acum2)
	print("Recorrido: ")
	print (camino2[0][0])
	k = 0
	while k < len(camino2):
		print(camino2[k][1])
		k += 1
	print ("Camino seleccionado: ")
	if acum1 > acum2:
		print("2")
	else:
		print("1")
	return 0
