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
    if M == "F":
        convex_hull = convex_hull_brute(points)
    if M == "G":
        convex_hull = convex_hull_graham(points)
    if M == "D":
        convex_hull = convex_hull_DC(points)
    ## Acá hay que sacar que lado conviene e imprimirlo como pide el enunciado
    # if M == "F":
    #     camino1 = extraerCamino(points[0], points[1], convex_hull)
    #     camino2 = extraerCamino(points[0], points[1], convex_hull)
    #     elegirCamino(camino1, camino2)
    # else:
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
    if len(points) < 3:
        return points
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
            x = points[i][0] - pivot[0]
            y = points[i][1] - pivot[1]
            angle_point = math.atan2(y,x)
            points_full_info.append([i,points[i],angle_point,x**2+y**2])
    points_full_info.sort(key=lambda x:x[3])
    points_full_info.sort(key=lambda x:x[2])
    # Si hay empate por ángulo, avanzan de a tamaño
    # Los dejo en points para no hacerme quilombo con los índices
    # Busco si hay más de uno con el primer ángulo, y con el último,
    # si eso pasa, ordeno de menor a mayor 
    # i = 1 # (Esto es para eliminar más de uno en el mismo ángulo. Ahora no lo usamos)
    # while i<len(points_full_info):
    #     if points_full_info[i][2] == points_full_info[i-1][2]:
    #         # Igual angulo, me fijo cual de los 2 saco
    #         if points_full_info[i][3] < points_full_info[i-1][3]:
    #             points_full_info.pop(i)
    #         else:
    #             points_full_info.pop(i-1)
    #     else:
    #         # si saco algo, la lista se modifica y no debo avanzar
    #         i=i+1

    # Ahora voy a ir agregando a la pila y fijándome si no hacen giros al lado
    # contrario. Si lo hacen saco hasta que deje de ocurrir
    # Arranco con los 3 primeros en la pila
    # Voy a usar la lista como stack, ya que agregar o sacar al final son O(1)
    convex_hull_points.append(points_full_info[0][1])
    convex_hull_points.append(points_full_info[1][1])
    for i in range(2,len(points_full_info)):
        current = points_full_info[i][1]
        while cross_product(current,convex_hull_points[-1],convex_hull_points[-2])>0.000001: # Comparo contra esto por errores numéricos
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
    p.sort(key=lambda x:-x[1])
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

    # Separo casos base
    if len(CHL)==1 and len(CHR)==1:
        return CHL+ CHR
    if len(CHL) == 0 or len(CHR) == 0:
        return CHL + CHR

    # Busco el punto de mayor coordenada X en el de la izquierda
    maxX =CHL[0][0]
    start_CHL = 0
    for i in range(0,len(CHL)):
        if maxX < CHL[i][0]:
            maxX = CHL[i][0]
            start_CHL = i

    # Siempre empiezo a recorrer el de la izquierda por start_CHL
    # En middle está el punto medio entre los cascos
    middle = (CHL[start_CHL][0]+CHR[0][0])/2

    # Separo casos que haya puntos en la recta del medio.
    # Si solo hay puntos en el medio, ese es el convex hull
    # Si hay puntos en el medio y puntos de solo uno de los lados, van los 
    # puntos en el medio en combinación con los de los lados
    # Si hay puntos en el medio y en ambos lados, me quedo con los puntos en los extremos
    i = 0
    num_left_middle_points = 0
    start_left_middle_points = -1
    while i<len(CHL): 
        if CHL[i][0] == middle:
            num_left_middle_points = num_left_middle_points+1
            if start_left_middle_points == -1:
                start_left_middle_points = i
        i = i+1

    i = 0
    num_right_middle_points = 0
    start_right_middle_points = -1
    while i<len(CHR): 
        if CHR[i][0] == middle:
            num_right_middle_points= num_right_middle_points +1
            if start_right_middle_points== -1:
                start_right_middle_points = i
        i = i+1

    if num_left_middle_points == len(CHL) and num_right_middle_points == len(CHR):
        return CHL + CHR
    if num_left_middle_points == len(CHL):
        # Cuando uno de los CH son todos puntos en el medio y el otro no, busco
        # los que están en el medio del otro (en este caso CHR) y los pongo en
        # el de la izquierda. Voy a tener uno que son todos los que están en el medio
        # y el otro que es los que no, y eso funciona después
        middle_points = CHL
        i = start_right_middle_points
        num_points_added = 0
        while num_points_added < num_right_middle_points:
            if CHR[i][0] == middle:
                middle_points.append(CHR[i])
                num_points_added = num_points_added+1
            i = i+1
        if num_points_added!=0:
            aux = []
            for i in range(0,len(CHR)):
                if CHR[i][0] != middle:
                    aux.append(CHR[i])
            CHR = aux
        CHL = middle_points
    elif num_right_middle_points == len(CHR):
        middle_points = CHR
        i = start_left_middle_points
        num_points_added = 0
        while num_points_added < num_left_middle_points:
            if CHL[i][0] == middle:
                middle_points.append(CHL[i])
                num_points_added = num_points_added+1
            i = i+1
        if num_points_added!=0:
            aux = []
            for i in range(0,len(CHL)):
                if CHL[i][0] != middle:
                    aux.append(CHL[i])
            CHL = aux
        CHR = middle_points
    elif num_left_middle_points != 0 and num_right_middle_points !=0:
        # El primero que aparece a la izquierda tiene que ser el mayor, y
        # ese es el único que me quedo. A la izquierda pasa eso con el último
        i = 0
        num_found = 0
        while i<len(CHL) and num_found< num_left_middle_points:
            if CHL[i][0] == middle:
                if num_found != 0:
                    CHL.remove(i)
                    i = i-1
                num_found = num_found+1
            i = i+1
        i = 0
        num_found = 0
        while i<len(CHR) and num_found< num_right_middle_points:
            if CHR[i][0] == middle:
                if num_found != num_right_middle_points-1:
                    CHL.remove(i)
                    i = i-1
                num_found = num_found+1
            i = i+1
    if len(CHL)==1 and len(CHR)==1:
        return CHL+ CHR
    if len(CHL) == 0 or len(CHR) == 0:
        return CHL + CHR

    # Busco el punto de mayor coordenada X en el de la izquierda
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
    y_m = intersect(CHL[l],CHR[r],middle,'upper')
    y_l = intersect(CHL[l_next],CHR[r],middle,'upper')
    y_r = intersect(CHL[l],CHR[r_next],middle,'upper')
    while (y_m<=y_l) or (y_m<=y_r):
        if y_m<y_l:
            y_m = y_l
            l = l_next
            l_next = (l+1)%len(CHL)
            y_l = intersect(CHL[l_next],CHR[r],middle,'upper')
            y_r = intersect(CHL[l],CHR[r_next],middle,'upper')
        elif y_m == y_l:
            # Esto pasa si hay uno justo en el borde
            if (CHL[l][1] < CHL[l_next][1]):
                y_m = y_l
                l = l_next
                l_next = (l+1)%len(CHL)
                y_l = intersect(CHL[l_next],CHR[r],middle,'upper')
                y_r = intersect(CHL[l],CHR[r_next],middle,'upper')
            else:
                y_l = y_l-0.1
        elif y_m<y_r:
            y_m = y_r
            r =r_next
            r_next = (r-1)%len(CHR)
            y_r = intersect(CHL[l],CHR[r_next],middle,'upper')
            y_l = intersect(CHL[l_next],CHR[r],middle,'upper')
        elif y_m == y_r:
            if (CHR[r][1] < CHR[r_next][1]):
                y_m = y_r
                r =r_next
                r_next = (r-1)%len(CHR)
                y_r = intersect(CHL[l],CHR[r_next],middle,'upper')
                y_l = intersect(CHL[l_next],CHR[r],middle,'upper')
            else:
                y_r = y_r-0.1
    upper_tangent = [l,r]

    # Tangente inferior
    l = start_CHL
    r = 0
    l_next = (l-1)%len(CHL)
    r_next = (r+1)%len(CHR)
    y_m = intersect(CHL[l],CHR[r],middle,'lower')
    y_l = intersect(CHL[l_next],CHR[r],middle,'lower')
    y_r = intersect(CHL[l],CHR[r_next],middle,'lower')
    while (y_m>=y_l) or (y_m>=y_r):
        if y_m>y_l:
            y_m = y_l
            l = l_next
            l_next = (l-1)%len(CHL)
            y_l = intersect(CHL[l_next],CHR[r],middle,'lower')
            y_r = intersect(CHL[l],CHR[r_next],middle,'lower')
        elif y_m == y_l:
            # Esto pasa si hay uno justo en el borde
            if (CHL[l][1] > CHL[l_next][1]):
                y_m = y_l
                l = l_next
                l_next = (l-1)%len(CHL)
                y_l = intersect(CHL[l_next],CHR[r],middle,'lower')
                y_r = intersect(CHL[l],CHR[r_next],middle,'lower')
            else:
                y_l = y_l+0.1

        elif y_m>y_r:
            y_m = y_r
            r =r_next
            r_next = (r+1)%len(CHR)
            y_r = intersect(CHL[l],CHR[r_next],middle,'lower')
            y_l = intersect(CHL[l_next],CHR[r],middle,'lower')
        elif y_m == y_r:
            if (CHR[r][1] > CHR[r_next][1]):
                y_m = y_r
                r =r_next
                r_next = (r+1)%len(CHR)
                y_r = intersect(CHL[l],CHR[r_next],middle,'lower')
                y_l = intersect(CHL[l_next],CHR[r],middle,'lower')
            else:
                y_r = y_r+0.1
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

def intersect(a,b,x,side):
    # Da el valor de y de la intersección de la recta definida por a,b a la
    # coordenada x 
    # Se asume que no están en una linea vertical a y b
    # Si lo están, devuelve el menor o mayor según si el side es upper o lower
    if b[0] == a[0]:
        if side == 'upper':
            return min(b[1],a[1])
        else:
            return max(b[1],a[1])
    m = (b[1]-a[1])/(b[0]-a[0])
    return m*(x-a[0])+a[1]

def convex_hull_brute(points):
    # Uso el primero y me fijo con cual pueden formar una recta que ponga todos
    # los puntos de un solo lado
    if len(points) < 3:
        return points
    finished = False
    CH = []
    current_point_to_check = points[0]
    while not finished:
        i = 0
        is_in_convex_hull = False
        while i<len(points) and is_in_convex_hull == False:
            # Me fijo si forman una arista del convex hull
            # Esto lo hago mirando si el producto cruz es siempre negativo con 
            if current_point_to_check == points[i]:
                i = i+1
            j = 0
            is_in_convex_hull = True
            colineal_points = []
            while is_in_convex_hull == True and j<len(points): 
                # Compruebo que todos los puntos estén a un costado
                while j<len(points) and(current_point_to_check == points[j] or points[i] == points[j]):
                    j=j+1
                if j == len(points):
                    break # No es lindo pero es la unica forma que se me ocurre
                product = cross_product(points[j],points[i],current_point_to_check)
                if product > 0:
                    is_in_convex_hull = False
                elif product == 0:
                    if points[j] != points[i] and points[j] != current_point_to_check:
                        colineal_points.append(points[j])
                j=j+1
            if is_in_convex_hull == True:
                # Me fijo si no hay otro colineal que esté más cerca
                min_distance = distance_between(points[i] , current_point_to_check)
                min_distance_point = points[i]
                for j in range(0,len(colineal_points)):
                    if distance_between(colineal_points[j],current_point_to_check) < min_distance:
                        min_distance_point = colineal_points[j]

                if len(CH) == 0:
                    # No hay nada en el convex hull, van los dos
                    CH.append(current_point_to_check)
                    CH.append(min_distance_point)
                elif min_distance_point == CH[0]:
                    finished = True
                else:
                    CH.append(min_distance_point)
            i=i+1
        if len(CH) == 0:
            import random
            current_point_to_check = random.choice(points)
        else:
            current_point_to_check = min_distance_point
    return CH








if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='parser.')
    parser.add_argument('file')
    parser.add_argument('method',choices=['F', 'G' , 'D'])
    parsed_args = parser.parse_args()
    points_file = parsed_args.file
    M = parsed_args.method
    main(points_file,M)
