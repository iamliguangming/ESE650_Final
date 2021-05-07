import heapq
from heapq import heappush, heappop  # Recommended.
import numpy as np
from math import sqrt
import time as t

from flightsim.world import World
from occupancy_map import OccupancyMap # Recommended.


def graph_search(world, resolution, margin, start, goal, astar):
    """
    Parameters:
        world,      World object representing the environment obstacles
        resolution, xyz resolution in meters for an occupancy map, shape=(3,)
        margin,     minimum allowed distance in meters from path to obstacles.
        start,      xyz position in meters, shape=(3,)
        goal,       xyz position in meters, shape=(3,)
        astar,      if True use A*, else use Dijkstra
    Output:
        path,       xyz position coordinates along the path in meters with
                    shape=(N,3). These are typically the centers of visited
                    voxels of an occupancy map. The first point must be the
                    start and the last point must be the goal. If no path
                    exists, return None.
    """

    global occ_map, start_index, goal_index
    global always_added,forced_to_add,forced_to_check,current_situation
    global new_x_g, new_y_g, new_z_g
    getNeibArray()

  
    # global always_added, global forced_to_check, global forced_to_add = getNeibArray()
    # current_situation = np.array([[26,0],[1,8],[3,12],[7,12]])

    # While not required, we have provided an occupancy map you may use or modify.
    star = t.time()
    occ_map = OccupancyMap(world, resolution, margin)
    print(t.time()-star)

    # Retrieve the index in the occupancy grid matrix corresponding to a position in space.
    start_index = tuple(occ_map.metric_to_index(start))
    goal_index = tuple(occ_map.metric_to_index(goal))
    visited_Points = set()

    occ_map._init_map_from_world
    Q = []

    cost_to_come = np.full((occ_map.map.shape[0],occ_map.map.shape[1],occ_map.map.shape[2]),np.inf)
    parent = np.zeros((occ_map.map.shape[0],occ_map.map.shape[1],occ_map.map.shape[2],3),dtype = int)





    if astar == False:
        cost_to_come[start_index[0],start_index[1],start_index[2]] = 0

        heappush(Q,(cost_to_come[start_index[0],start_index[1],start_index[2]],[start_index[0],start_index[1],start_index[2]]))
        heappush(Q,(cost_to_come[goal_index[0],goal_index[1],goal_index[2]],[goal_index[0],goal_index[1],goal_index[2]]))
        #Done with initialization
        #Following is the searching process

        while (cost_to_come[goal_index[0],goal_index[1],goal_index[2]],[goal_index[0],goal_index[1],goal_index[2]]) in Q and cost_to_come.min() < np.inf:
            u_x,u_y,u_z = heappop(Q)[1]
            visited_Points.add((u_x,u_y,u_z))
            for i in range(-1,2):
                for j in range(-1,2):
                    for k in range(-1,2):
                        if not occ_map.is_valid_index([u_x+i,u_y+j,u_z+k]) or occ_map.is_occupied_index([u_x+i,u_y+j,u_z+k]) or (u_x+i,u_y+j,u_z+k) in visited_Points:
                            pass
                        else:
                            d = cost_to_come[u_x,u_y,u_z]+sqrt(i**2 + j**2 +k**2)
                            if d < cost_to_come[u_x+i,u_y+j,u_z+k]:
                                cost_to_come[u_x+i,u_y+j,u_z+k] = d
                                heappush(Q,(cost_to_come[u_x+i,u_y+j,u_z+k],[u_x+i,u_y+j,u_z+k]))
                                parent[u_x+i,u_y+j,u_z+k,:] = [u_x,u_y,u_z]
    elif astar == True:
        heuristic = np.full((occ_map.map.shape[0],occ_map.map.shape[1],occ_map.map.shape[2]),np.inf)

        for i in range(occ_map.map.shape[0]):
           for j in range(occ_map.map.shape[1]):
             for k in range(occ_map.map.shape[2]):
               heuristic[i,j,k] = getHeur([i,j,k],resolution)
        f = cost_to_come + heuristic

        cost_to_come[start_index[0],start_index[1],start_index[2]] = 0
        parent[start_index[0],start_index[1],start_index[2],:] = start_index[0],start_index[1],start_index[2]
        f[start_index[0],start_index[1],start_index[2]] = 0 + heuristic[start_index[0],start_index[1],start_index[2]]

        heappush(Q,(f[start_index[0],start_index[1],start_index[2]],[start_index[0],start_index[1],start_index[2]]))
        heappush(Q,(f[goal_index[0],goal_index[1],goal_index[2]],[goal_index[0],goal_index[1],goal_index[2]]))
        #Done with initialization
        #Following is the searching process

        while (f[goal_index[0],goal_index[1],goal_index[2]],[goal_index[0],goal_index[1],goal_index[2]]) in Q and min(Q)[0] < np.inf:

            u_indicies = heappop(Q)[1]
            # print(u_indicies)
            u_x = u_indicies[0]
            u_y = u_indicies[1]
            u_z = u_indicies[2]
            # if [u_x,u_y,u_z] == [19,39,4]:
            #     pass
            p_x,p_y,p_z= parent[int(u_x),int(u_y),int(u_z),:]
            dif = np.array(u_indicies,dtype=int) - np.array([p_x,p_y,p_z],dtype =int)
            for i in range(3):
                if dif[i]==0:
                    dif[i]=0
                else:
                    dif[i] = abs(dif[i])/dif[i]
            dx,dy,dz = dif





##dx dy dz not updated correctly
            norm1 = abs(np.array([dx,dy,dz])).sum()
            num_neib = current_situation[norm1][0]
            num_fneib = current_situation[norm1][1]
            ID = int(dx+1 + 3*(dy+1) +9*(dz+1))
            for i in range(num_neib+num_fneib):
                if i < num_neib:
                    dx = always_added[ID][0][i]
                    dy = always_added[ID][1][i]
                    dz = always_added[ID][2][i]
                    if not jump(u_x,u_y,u_z,dx,dy,dz,True):
                        continue

                else:
                    nx = u_x + forced_to_check[ID][0][i-num_neib]
                    ny = u_y + forced_to_check[ID][1][i-num_neib]
                    nz = u_z + forced_to_check[ID][2][i-num_neib]
                    if occ_map.is_valid_index([nx,ny,nz]) and occ_map.is_occupied_index([nx,ny,nz]):
                        dx = forced_to_add[ID][0][i-num_neib]
                        dy = forced_to_add[ID][1][i-num_neib]
                        dz = forced_to_add[ID][2][i-num_neib]
                        if not jump(u_x,u_y,u_z,dx,dy,dz,True):
                            continue
                    else:
                        continue
                d = cost_to_come[u_x,u_y,u_z ]+ sqrt(((new_x_g-u_x)*resolution[0])**2 + ((new_y_g-u_y)*resolution[1])**2+((new_z_g-u_z)*resolution[2])**2)
                if d < cost_to_come[new_x_g,new_y_g,new_z_g]:
                    cost_to_come[new_x_g,new_y_g,new_z_g]=d
                    f[new_x_g,new_y_g,new_z_g] = d + heuristic[new_x_g,new_y_g,new_z_g]
                    heappush(Q,(f[new_x_g,new_y_g,new_z_g],[new_x_g,new_y_g,new_z_g]))
                    parent[new_x_g,new_y_g,new_z_g] = [u_x,u_y,u_z]

    current_X = goal_index[0]
    current_Y = goal_index[1]
    current_Z = goal_index[2]



    path = []
    if parent[goal_index[0],goal_index[1],goal_index[2],:].any() == 0:
        return None


    while [current_X,current_Y,current_Z] != [start_index[0],start_index[1],start_index[2]]:
        path.insert(0,occ_map.index_to_metric_center([current_X,current_Y,current_Z]))
        [current_X,current_Y,current_Z] = parent[int(current_X),int(current_Y),int(current_Z),:]
    # path.insert(0,occ_map.index_to_metric_center(start_index))
    path.insert(0,start)
    path.append(goal)

    return np.asarray(path)











    # if astar == False:
    #     pass




    return None

def getNeighbors(dx,dy,dz, runs ):
    norm = abs(np.array([dx,dy,dz])).sum()
    if norm == 0:
        if runs == 0:tx,ty,tz = 1,0,0
        elif runs == 1: tx=-1; ty=0; tz=0
        elif runs == 2: tx=0; ty=1; tz=0
        elif runs == 3: tx=1; ty=1; tz=0
        elif runs == 4: tx=-1; ty=1; tz=0
        elif runs == 5: tx=0; ty=-1; tz=0
        elif runs == 6: tx=1; ty=-1; tz=0
        elif runs == 7: tx=-1; ty=-1; tz=0
        elif runs == 8: tx=0; ty=0; tz=1
        elif runs == 9: tx=1; ty=0; tz=1
        elif runs == 10: tx=-1; ty=0; tz=1
        elif runs == 11: tx=0; ty=1; tz=1
        elif runs == 12: tx=1; ty=1; tz=1
        elif runs == 13: tx=-1; ty=1; tz=1
        elif runs == 14: tx=0; ty=-1; tz=1
        elif runs == 15: tx=1; ty=-1; tz=1
        elif runs == 16: tx=-1; ty=-1; tz=1
        elif runs == 17: tx=0; ty=0; tz=-1
        elif runs == 18: tx=1; ty=0; tz=-1
        elif runs == 19: tx=-1; ty=0; tz=-1
        elif runs == 20: tx=0; ty=1; tz=-1
        elif runs == 21: tx=1; ty=1; tz=-1
        elif runs == 22: tx=-1; ty=1; tz=-1
        elif runs == 23: tx=0; ty=-1; tz=-1
        elif runs == 24: tx=1; ty=-1; tz=-1
        elif runs == 25: tx=-1; ty=-1; tz=-1
    elif norm == 1:
        tx = dx; ty =dy; tz = dz
    elif norm == 2:
        if runs == 0:
            if dz == 0 :
                tx,ty,tz = 0,dy,0
            else:
                tx,ty,tz = 0,0,dz
        elif runs ==1:
            if dx == 0:
                tx,ty,tz = 0,dy,0
            else:
                tx,ty,tz = dx,0,0
        elif runs == 2:
            tx,ty,tz = dx,dy,dz
    elif norm == 3:
        if runs == 0: tx = dx; ty =  0; tz =  0
        elif runs == 1: tx =  0; ty = dy; tz =  0
        elif runs == 2: tx =  0; ty =  0; tz = dz
        elif runs == 3: tx = dx; ty = dy; tz =  0
        elif runs == 4: tx = dx; ty =  0; tz = dz
        elif runs == 5: tx =  0; ty = dy; tz = dz
        elif runs == 6: tx = dx; ty = dy; tz = dz
    return [tx,ty,tz]


def getForcedN(dx,dy,dz, runs):

    norm = abs(np.array([dx,dy,dz])).sum()
    if norm == 1:
        if runs == 0: fx= 0; fy= 1; fz = 0
        elif runs == 1: fx= 0; fy=-1; fz = 0
        elif runs == 2: fx= 1; fy= 0; fz = 0
        elif runs == 3: fx= 1; fy= 1; fz = 0
        elif runs == 4: fx= 1; fy=-1; fz = 0
        elif runs == 5: fx=-1; fy= 0; fz = 0
        elif runs == 6: fx=-1; fy= 1; fz = 0
        elif runs == 7: fx=-1; fy=-1; fz = 0
        nx,ny,nz = fx,fy,fz
        if(dx != 0):
            fz = fx; fx = 0
            nz = fz; nx = dx
        if(dy != 0):
            fz = fy; fy = 0
            nz = fz; ny = dy
    elif norm == 2:
        if dx == 0:
          if runs == 0:
            fx = 0; fy = 0; fz = -dz
            nx = 0; ny = dy; nz = -dz

          elif runs == 1:
            fx = 0; fy = -dy; fz = 0
            nx = 0; ny = -dy; nz = dz

          elif runs == 2:
            fx = 1; fy = 0; fz = 0
            nx = 1; ny = dy; nz = dz

          elif runs == 3:
            fx = -1; fy = 0; fz = 0
            nx = -1; ny = dy; nz = dz

          elif runs == 4:
            fx = 1; fy = 0; fz = -dz
            nx = 1; ny = dy; nz = -dz

          elif runs == 5:
            fx = 1; fy = -dy; fz = 0
            nx = 1; ny = -dy; nz = dz

          elif runs == 6:
            fx = -1; fy = 0; fz = -dz
            nx = -1; ny = dy; nz = -dz

          elif runs == 7:
            fx = -1; fy = -dy; fz = 0
            nx = -1; ny = -dy; nz = dz


          elif runs == 8:
            fx = 1; fy = 0; fz = 0
            nx = 1; ny = dy; nz = 0

          elif runs == 9:
            fx = 1; fy = 0; fz = 0
            nx = 1; ny = 0; nz = dz

          elif runs == 10:
            fx = -1; fy = 0; fz = 0
            nx = -1; ny = dy; nz = 0

          elif runs == 11:
            fx = -1; fy = 0; fz = 0
            nx = -1; ny = 0; nz = dz

        elif dy ==0:
          if runs == 0:
            fx = 0; fy = 0; fz = -dz
            nx = dx; ny = 0; nz = -dz

          elif runs == 1:
            fx = -dx; fy = 0; fz = 0
            nx = -dx; ny = 0; nz = dz

          elif runs == 2:
            fx = 0; fy = 1; fz = 0
            nx = dx; ny = 1; nz = dz

          elif runs == 3:
            fx = 0; fy = -1; fz = 0
            nx = dx; ny = -1;nz = dz

          elif runs == 4:
            fx = 0; fy = 1; fz = -dz
            nx = dx; ny = 1; nz = -dz

          elif runs == 5:
            fx = -dx; fy = 1; fz = 0
            nx = -dx; ny = 1; nz = dz

          elif runs == 6:
            fx = 0; fy = -1; fz = -dz
            nx = dx; ny = -1; nz = -dz

          elif runs == 7:
            fx = -dx; fy = -1; fz = 0
            nx = -dx; ny = -1; nz = dz

          # // Extras
          elif runs == 8:
            fx = 0; fy = 1; fz = 0
            nx = dx; ny = 1; nz = 0

          elif runs == 9:
            fx = 0; fy = 1; fz = 0
            nx = 0; ny = 1; nz = dz

          elif runs == 10:
            fx = 0; fy = -1; fz = 0
            nx = dx; ny = -1; nz = 0

          elif runs == 11:
            fx = 0; fy = -1; fz = 0
            nx = 0; ny = -1; nz = dz
        else:
          if runs == 0:
            fx = 0; fy = -dy; fz = 0
            nx = dx; ny = -dy; nz = 0

          elif runs == 1:
            fx = -dx; fy = 0; fz = 0
            nx = -dx; ny = dy; nz = 0

          elif runs == 2:
            fx =  0; fy = 0; fz = 1
            nx = dx; ny = dy; nz = 1

          elif runs == 3:
            fx =  0; fy = 0; fz = -1
            nx = dx; ny = dy; nz = -1

          elif runs == 4:
            fx = 0; fy = -dy; fz = 1
            nx = dx; ny = -dy; nz = 1

          elif runs == 5:
            fx = -dx; fy = 0; fz = 1
            nx = -dx; ny = dy; nz = 1

          elif runs == 6:
            fx = 0; fy = -dy; fz = -1
            nx = dx; ny = -dy; nz = -1

          elif runs == 7:
            fx = -dx; fy = 0; fz = -1
            nx = -dx; ny = dy; nz = -1


          elif runs == 8:
            fx =  0; fy = 0; fz = 1
            nx = dx; ny = 0; nz = 1

          elif runs == 9:
            fx = 0; fy = 0; fz = 1
            nx = 0; ny = dy; nz = 1

          elif runs == 10:
            fx =  0; fy = 0; fz = -1
            nx = dx; ny = 0; nz = -1

          elif runs == 11:
            fx = 0; fy = 0; fz = -1
            nx = 0; ny = dy; nz = -1
    elif norm ==3:
        if runs == 0:
          fx = -dx; fy = 0; fz = 0
          nx = -dx; ny = dy; nz = dz

        elif runs == 1:
          fx = 0; fy = -dy; fz = 0
          nx = dx; ny = -dy; nz = dz

        elif runs == 2:
          fx = 0; fy = 0; fz = -dz
          nx = dx; ny = dy; nz = -dz

        # // Need to check up to here for forced!
        elif runs == 3:
          fx = 0; fy = -dy; fz = -dz
          nx = dx; ny = -dy; nz = -dz

        elif runs == 4:
          fx = -dx; fy = 0; fz = -dz
          nx = -dx; ny = dy; nz = -dz

        elif runs == 5:
          fx = -dx; fy = -dy; fz = 0
          nx = -dx; ny = -dy; nz = dz

        # // Extras
        elif runs == 6:
          fx = -dx; fy = 0; fz = 0
          nx = -dx; ny = 0; nz = dz

        elif runs == 7:
          fx = -dx; fy = 0; fz = 0
          nx = -dx; ny = dy; nz = 0

        elif runs == 8:
          fx = 0; fy = -dy; fz = 0
          nx = 0; ny = -dy; nz = dz

        elif runs == 9:
          fx = 0; fy = -dy; fz = 0
          nx = dx; ny = -dy; nz = 0

        elif runs == 10:
          fx = 0; fy = 0; fz = -dz
          nx = 0; ny = dy; nz = -dz

        elif runs == 11:
          fx = 0; fy = 0; fz = -dz
          nx = dx; ny = 0; nz = -dz
    return [fx,fy,fz,nx,ny,nz]



def getNeibArray():

    global always_added,forced_to_add,forced_to_check,current_situation
    always_added  = np.zeros((27,3,26),dtype = int)
    forced_to_check = np.zeros((27,3,12),dtype = int)
    forced_to_add = np.zeros((27,3,12),dtype = int)
    current_situation = np.array([[26,0],[1,8],[3,12],[7,12]])
    ID = 0
    for dz in range(-1,2):
        for dy in range(-1,2):
            for dx in range (-1,2):
                norm1 = int(abs(np.array([dx,dy,dz])).sum())
                for runs in range(current_situation[norm1][0]):
                    always_added[ID][0][runs],always_added[ID][1][runs],always_added[ID][2][runs] = getNeighbors(dx, dy, dz, runs)
                for runs in range(current_situation[norm1][1]):
                    forced_to_check[ID][0][runs],forced_to_check[ID][1][runs],forced_to_check[ID][2][runs],forced_to_add[ID][0][runs],forced_to_add[ID][1][runs],forced_to_add[ID][2][runs]=getForcedN(dx, dy, dz, runs)
                ID += 1
    return None



def jump(x,y,z,dx,dy,dz,status):
    global new_x_g, new_y_g,new_z_g
    new_x = x + dx
    new_y = y+ dy
    new_z = z + dz
    if not occ_map.is_valid_index([new_x,new_y,new_z]) or occ_map.is_occupied_index([new_x,new_y,new_z]):
        return False
    if new_x == goal_index[0] and new_y == goal_index[1] and new_z == goal_index[2]:
        new_x_g,new_y_g,new_z_g= new_x,new_y,new_z
        return True
    if (hasForced(new_x,new_y,new_z,dx,dy,dz,False)):
        new_x_g,new_y_g,new_z_g= new_x,new_y,new_z
        return True
    new_x_g,new_y_g,new_z_g= new_x,new_y,new_z
    ID = (dx+1) + 3*(dy+1 ) + 9*(dz+1)
    norm1 = abs(dx) + abs(dy) + abs(dz)
    num_neib = current_situation[norm1][0]

    for i in range(num_neib-1):
          new_new_x,new_new_y,new_new_z = new_x,new_y,new_z
          if jump(new_new_x,new_new_y,new_new_z,always_added[ID][0][i],always_added[ID][1][i],always_added[ID][2][i],False):
              new_x_g,new_y_g,new_z_g= new_x,new_y,new_z
              return True

    return jump(new_x,new_y,new_z,dx,dy,dz,True)






def hasForced(x,y,z,dx,dy,dz,status):
    norm1 = abs(dx) + abs(dy) + abs(dz)
    ID = (dx+1) + 3*(dy+1 ) + 9*(dz+1)
    if norm1 == 1:
        for fn in range(8):
            nx = x + forced_to_check[ID][0][fn]
            ny = y + forced_to_check[ID][1][fn]
            nz = z + forced_to_check[ID][2][fn]
            if occ_map.is_valid_index([nx,ny,nz]) and occ_map.is_occupied_index([nx,ny,nz]):
                   return True
        return False
    if norm1 ==2:
        for fn in range(8):
            nx = x + forced_to_check[ID][0][fn]
            ny = y + forced_to_check[ID][1][fn]
            nz = z + forced_to_check[ID][2][fn]
            if occ_map.is_valid_index([nx,ny,nz]) and occ_map.is_occupied_index([nx,ny,nz]):
                   return True
        return False
    if norm1 ==3:
        for fn in range(6):
            nx = x + forced_to_check[ID][0][fn]
            ny = y + forced_to_check[ID][1][fn]
            nz = z + forced_to_check[ID][2][fn]
            if occ_map.is_valid_index([nx,ny,nz]) and occ_map.is_occupied_index([nx,ny,nz]):
                   return True
        return False
    return False

def getHeur(index,resolution):
    return sqrt(((goal_index[0]-index[0])*resolution[0])**2+((goal_index[1]-index[1])*resolution[1])**2+((goal_index[2]-index[2])*resolution[2])**2)



