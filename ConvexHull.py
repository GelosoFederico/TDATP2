# -*- coding: utf-8 -*-
# TP2 TDA Parte 1: El robot y el camino seguro
# Se dan un conjunto de N puntos en un archivo, de los cuales hay uno de inicio
# y uno de fin
# Se pide calcular el camino más corto que recorra sobre la envoltura convexa
# que vaya de inicio a fin.
# Se proponen 3 métodos para calcular la envoltura convexa: A fuerza bruta, con
# Graham scan y con D&C (-F , -G o -D)

import argparse
import math
import brute

def main(points_file,M):
    ## Interpreto el archivo
    # En la posición 0 de cada punto está la coordenada X, en la 1 es la Y
    points = read_points(points_file)
    print(points)
    M = "D"
    if M == "F":
        convex_hull = brute.fuerzabruta(points)
    if M == "G":
        convex_hull = convex_hull_graham(points)
    if M == "D":
        convex_hull = convex_hull_DC(points)
    ## Acá hay que sacar que lado conviene e imprimirlo como pide el enunciado
    # print(points)
    if M == "F":
        camino1 = extraerCamino(points[0], points[1], convex_hull)
        camino2 = extraerCamino(points[0], points[1], convex_hull)
        elegirCamino(camino1, camino2)
    else:
        print_path(points,convex_hull)

    return 

def read_points(str_file):
    ## Abro el archivo y guardo los puntos en una lista
    points_list = []
    for line in open(str_file, 'r'):
        new_point = []
        for i in line.split(' '):
            new_point.append(float(i))
        points_list.append(new_point)
    return points_list

def convex_hull_graham(points):
    # Busco el punto con coordenada Y mínima (más abajo). En caso de empate es
    # el más a la izquierda (menor X)
    # Ordeno por ángulo, yendo en sentido antihorario tomando como eje el punto
    # de antes
    convex_hull_points = []
    pivot = points[0]
    for i in range(1,len(points)):
        if points[i][1] < pivot[1]:
            pivot = points[i]
        elif points[i][1] == pivot[1] and points[i][0] < pivot[0]:
            pivot = points[i]
    # Ordeno por ángulo
    convex_hull_points.append(pivot)
    # points_full_info tiene posicion en points, el punto en si, cos y modulo
    points_full_info = []
    for i in range(0,len(points)):
        if pivot != points[i]:
            x = (pivot[0] - points[i][0])**2
            y = (pivot[1] - points[i][1])**2
            cos_point = x/(x+y)
            points_full_info.append([i,points[i],cos_point,x+y])
    points_full_info.sort(key=lambda x:x[2])
    # Saco los que tengan el mismo ángulo, me quedo con el de máximo módulo
    # Los dejo en points para no hacerme quilombo con los índices
    i = 1
    while i<len(points_full_info):
        if points_full_info[i][2] == points_full_info[i-1][2]:
            # Igual angulo, me fijo cual de los 2 saco
            if points_full_info[i][3] < points_full_info[i-1][3]:
                points_full_info.pop(i)
            else:
                points_full_info.pop(i-1)
        else:
            # si saco algo, la lista se modifica y no debo avanzar
            i=i+1

    # Ahora voy a ir agregando a la pila y fijándome si no hacen giros al lado
    # contrario. Si lo hacen saco hasta que deje de ocurrir
    # Arranco con los 3 primeros en la pila
    # Voy a usar la lista como stack, ya que agregar o sacar al final son O(1)
    convex_hull_points.append(points_full_info[0][1])
    convex_hull_points.append(points_full_info[1][1])
    for i in range(2,len(points_full_info)):
        current = points_full_info[i][1]
        while cross_product(current,convex_hull_points[-1],convex_hull_points[-2])<0:
            convex_hull_points.pop()
        convex_hull_points.append(current)
    return convex_hull_points

def cross_product(c,b,a):
    # Supongo dos segmentos de recta que van de a hacia c
    cb = [c[0] - b[0],c[1] - b[1]]
    ba = [b[0] - a[0],b[1] - a[1]]
    return (cb[0]*ba[1])-(cb[1]*ba[0])

def print_path(points,hull):
    # En points está en el primero el inicio y el último el de llegada
    # Hago los dos recorridos, y al final compruebo cual tiene menor distancia
    # Se espera que los puntos del hull estén en orden
    print(hull)
    start = points[0]
    finish = points[1]

    st_i =hull.index(start)
    last = hull[st_i]
    st_i = (st_i+1)%len(hull)
    distance = 0
    path = [last]
    while hull[st_i]!=finish:
        distance += distance_between(last,hull[st_i])
        last = hull[st_i]
        path.append(last)
        st_i = (st_i+1)%len(hull)
    path.append(hull[st_i])
    distance += distance_between(last,hull[st_i])
    print("Camino 1: Longitud " + str(distance))
    print(path)
    distance1 = distance

    st_i =hull.index(start)
    last = hull[st_i]
    st_i = st_i-1
    distance = 0
    path = [last]
    while hull[st_i]!=finish:
        distance += distance_between(last,hull[st_i])
        last = hull[st_i]
        path.append(last)
        st_i = (st_i-1)%len(hull)
    path.append(hull[st_i])
    distance += distance_between(last,hull[st_i])
    distance2 = distance
    print("Camino 2: Longitud " + str(distance))
    print(path)
    print("1" if distance1<distance2 else "2")
    return


def distance_between(a,b):
    return math.sqrt((b[0]-a[0])**2+(b[1]-a[1])**2)

def convex_hull_DC(points):
    # Se ordena según x, y se va dividiendo a partes iguales.
    # Se llama recursivamente para cada mitad y de cada uno devuelve el CH,
    # ordenados de forma antihoraria, empezando con el punto de menor x
    p = points[:]
    p.sort(key=lambda x:x[0])
    return _convex_hull_DC(p)

def _convex_hull_DC(points):
    if len(points) <= 2:
        return points
    points_left = points[0:len(points)//2]
    CH_L=_convex_hull_DC(points_left)
    points_right = points[len(points)//2:]
    CH_R=_convex_hull_DC(points_right)
    return _CH_merge(CH_L,CH_R)

def _CH_merge(CHL,CHR):
    # Los convex hull se suponen en sentido antihorario ambos, empezando por el
    # punto en el extremo izquierdo
    # Para unirlos, se verá cual es la tangente entre ambos polígonos, y se
    # descartaran los que están en el medio.
    # Para hallar los puntos que forman la tangente, se recorre tomando un punto
    # de cada lado y se busca cual pareja toca la linea vertical media más alto
    # y más bajo

    # Busco el punto de mayor coordenada X en el de la izquierda
    if len(CHL)==1 and len(CHR)==1:
        return CHL+ CHR
    maxX =CHL[0][0]
    start_CHL = 0
    for i in range(0,len(CHL)):
        if maxX < CHL[i][0]:
            maxX = CHL[i][0]
            start_CHL = i

    # Siempre empiezo a recorrer el de la izquierda por start_CHL
    # En middle está el punto medio entre los cascos
    middle = (CHL[start_CHL][0]+CHR[0][0])/2

    # Saco tangente superior
    # l y r son el punto en el que estoy parado en cada casco
    # l y r next son los siguientes, según su movimiento(antihorario y horario)
    # y_m es la intersección con el medio actual
    # y_r e y_l son las que ocurren con l o r next
    # Se busca que y_m sea maxima, que ocurre cuando y_r e y_l son menores 
    # ambas
    l = start_CHL
    r = 0
    l_next = (l+1)%len(CHL)
    r_next = (r-1)%len(CHR)
    y_m = intersect(CHL[l],CHR[r],middle)
    y_l = intersect(CHL[l_next],CHR[r],middle)
    y_r = intersect(CHL[l],CHR[r_next],middle)
    while (y_m<y_l) or (y_m<y_r):
        if y_m<y_l:
            y_m = y_l
            l = l_next
            l_next = (l+1)%len(CHL)
            y_l = intersect(CHL[l_next],CHR[r],middle)
            y_r = intersect(CHL[l],CHR[r_next],middle)
        elif y_m<y_r:
            y_m = y_r
            r =r_next
            r_next = (r-1)%len(CHR)
            y_r = intersect(CHL[l],CHR[r_next],middle)
            y_l = intersect(CHL[l_next],CHR[r],middle)
    upper_tangent = [l,r]

    # Tangente inferior
    l = start_CHL
    r = 0
    l_next = (l+1)%len(CHL)
    r_next = (r+1)%len(CHR)
    y_m = intersect(CHL[l],CHR[r],middle)
    y_l = intersect(CHL[l_next],CHR[r],middle)
    y_r = intersect(CHL[l],CHR[r_next],middle)
    while (y_m>y_l) or (y_m>y_r):
        if y_m>y_l:
            y_m = y_l
            l = l_next
            l_next = (l+1)%len(CHL)
            y_l = intersect(CHL[l_next],CHR[r],middle)
        elif y_m>y_r:
            y_m = y_r
            r =r_next
            r_next = (r+1)%len(CHR)
            y_r = intersect(CHL[l],CHR[r_next],middle)
    lower_tangent = [l,r]

    # Uno en orden antihorario
    CH = []
    i=0
    while i != lower_tangent[0]:
        CH.append(CHL[i])
        i +=1
    CH.append(CHL[i])
    j = lower_tangent[1]
    while j != upper_tangent[1]:
        CH.append(CHR[j])
        j = (j+1)%len(CHR)
    CH.append(CHR[j])
    # Así no lleno de más
    if upper_tangent[0] > i:
        i = upper_tangent[0]
        while i != len(CHL) and i != lower_tangent[0] :
            CH.append(CHL[i])
            i +=1
    return CH

def intersect(a,b,x):
    # Da el valor de y de la intersección de la recta definida por a,b a la
    # coordenada x 
    # Se asume que no están en una linea vertical a y b
    m = (b[1]-a[1])/(b[0]-a[0])
    return m*(x-a[0])+a[1]

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='parser.')
    parser.add_argument('file')
    parser.add_argument('method',choices=['F', 'G' , 'D'])
    parsed_args = parser.parse_args()
    points_file = parsed_args.file
    M = parsed_args.method
    main(points_file,M)
