/******************************************************************************************************************************
*                                                                                                                             *
   Team Id             : BV#901
   Author List         : Divyansh Malhotra, Chetan Chawla, Harshil Bansal,Ishani Janveja
   Filename            : eYRC-BV#901
   Theme               : Bothoven
   Functions           : calcSizeNe,calcsize,mapp,mapp2,findPath,deterPath,deleteNeighbour,Stop,Forw,Back,Left,Right,face,
                         Line_Follow_Forw,Line_Follow_Back,Front,Count_Right,Count_Left,Forw_Cm,Back_Cm,Traverse,Buzzer,
                         set,allign,Right_D,Left_D,Rev,Play
   Global Variables    : ShaftCountRight, ShaftCountLeft, Thresh, ls, ms, rs, mnp, mml, a,b....x,A,B,....X, bws, source,
                         path, neigh, beer, temp, ch, f1, len, reach, Curr_Pos, Next_Pos, Prev_Pos, clockwise, obstacle,
                         verify, weed, pos
*                                                                                                                             *
*******************************************************************************************************************************/

int mnp[] = {7, 29, 26, 18, 24, 13, 30, 16, 20};    /*Input array*/


char mml;
char a = 'a', b = 'b', c = 'c', d = 'd', e = 'e', f = 'f', g = 'g', h = 'h', i = 'i', j = 'j', k = 'k', l = 'l', m = 'm', n = 'n', o = 'o', p = 'p', q = 'q', r = 'r', s = 's', t = 't', u = 'u', v = 'v', w = 'w', x = 'x', A = 'A', B = 'B', C = 'C', D = 'D', E = 'E', F = 'F', G = 'G', H = 'H', I = 'I', J = 'J', K = 'K', L = 'L', M = 'M', N = 'N', O = 'O', P = 'P', Q = 'Q', R = 'R', S = 'S', T = 'T', U = 'U', V = 'V', W = 'W', X = 'X';
/*bws, source, path- arrays to implement breadth first search traversal */
char bws[48];
char source[48];
char path[48];
/*neigh[i1][j1]- stores the nodes that can approach the mnp i1+1*/
char neigh[33][7] = {{a, '0', '0', '0', '0', '0', '0'}, {b, '0', '0', '0', '0', '0', '0'}, {c, '0', '0', '0', '0', '0', '0'}, {d, '0', '0', '0', '0', '0', '0'}, {e, '0', '0', '0', '0', '0', '0'}, {f, '0', '0', '0', '0', '0', '0'}, {g, '0', '0', '0', '0', '0', '0'}, {h, '0', '0', '0', '0', '0', '0'}, {i, '0', '0', '0', '0', '0', '0'}, {j, '0', '0', '0', '0', '0', '0'}, {k, '0', '0', '0', '0', '0', '0'}, {l, '0', '0', '0', '0', '0', '0'}, {m, '0', '0', '0', '0', '0', '0'}, {n, '0', '0', '0', '0', '0', '0'}, {o, '0', '0', '0', '0', '0', '0'}, {p, '0', '0', '0', '0', '0', '0'}, {q, '0', '0', '0', '0', '0', '0'}, {r, '0', '0', '0', '0', '0', '0'}, {s, '0', '0', '0', '0', '0', '0'}, {t, '0', '0', '0', '0', '0', '0'}, {u, '0', '0', '0', '0', '0', '0'}, {v, '0', '0', '0', '0', '0', '0'}, {w, '0', '0', '0', '0', '0', '0'}, {x, '0', '0', '0', '0', '0', '0'}, {a, C, A, B, '0', '0', '0'}, {B, C, D, E, F, G, '0'}, {e, F, G, H, '0', '0', '0'}, {i, O, I, J, '0', '0', '0'}, {J, K, L, M, N, O, '0'}, {L, m, K, P, '0', '0', '0'}, {q, R, Q, S, '0', '0', '0'}, {R, S, T, U, V, W, '0'}, {u, U, T, X, '0', '0', '0'}};
/*beer[i1][j1]- stores the neighbours of a node in accordance to their mapped values(using mapp) as i1*/
char beer[50][5] = {{b, x, '0', '0', '0'}, {c, a, '0', '0', '0'}, {d, G, B, b, '0'}, {e, c, '0', '0', '0'}, {f, d, '0', '0', '0'}, {g, e, '0', '0', '0'}, {h, I, H, f, '0'}, {i, g, '0', '0', '0'}, {j, h, '0', '0', '0'}, {k, i, '0', '0', '0'}, {l, K, J, j, '0'}, {m, k, '0', '0', '0'}, {n, l, '0', '0', '0'}, {o, m, '0', '0', '0'}, {p, Q, P, n, '0'}, {q, o, '0', '0', '0'}, {r, p, '0', '0', '0'}, {s, q, '0', '0', '0'}, {t, T, S, r, '0'}, {u, s, '0', '0', '0'}, {v, t, '0', '0', '0'}, {w, u, '0', '0', '0'}, {x, A, X, v, '0'}, {a, w, '0', '0', '0'}, {C, w, '0', '0', '0'}, {c, G, C, '0', '0'}, {B, D, U, A, '0'}, {E, C, '0', '0', '0'}, {F, D, '0', '0', '0'}, {H, G, E, O, '0'}, {c, B, F, '0', '0'}, {g, F, '0', '0', '0'}, {g, O, '0', '0', '0'}, {O, K, k, '0', '0'}, {k, L, J, '0', '0'}, {K, P, M, R, '0'}, {L, N, '0', '0', '0'}, {M, O, '0', '0', '0'}, {J, N, F, I, '0'}, {L, o, '0', '0', '0'}, {R, o, '0', '0', '0'}, {S, W, L, Q, '0'}, {R, s, T, '0', '0'}, {U, S, s, '0', '0'}, {C, V, T, X, '0'}, {W, U, '0', '0', '0'}, {R, V, '0', '0', '0'}, {U, w, '0', '0', '0'}};
char temp[4];
char ch = 'a', f1 = 'n';
int len = 0, reach = 1;
int ShaftCountRight = 0, ShaftCountLeft = 0;                              /*ISR increaments*/
int Thresh = 48;                                                          /* White Line Sensor Threshhold Value*/
int ls = 0, ms = 0, rs = 0;                                               /* White Line Sensor Values*/
int Curr_Pos, Next_Pos, Prev_Pos;
int clockwise = 1, obstacle = 0, verify = 1, weed = 0, pos = 0;


/*
   Function Name      : calcSizeNe
   Input              : hmm
   Output             : Hard-coded integer values
   Logic              : Returns the number of nodes corresponding to a MNP(hmm)
   Example Call       : calcSizeNe(25)
*/
int calcSizeNe(int hmm)
{
  int ind = 0;
  if (hmm == 1 || hmm == 2 || hmm == 3 || hmm == 4 || hmm == 5 || hmm == 6 || hmm == 7 || hmm == 8 || hmm == 9 || hmm == 10 || hmm == 11 || hmm == 12 || hmm == 13 || hmm == 14 || hmm == 15 || hmm == 16 || hmm == 17 || hmm == 18 || hmm == 19 || hmm == 20 || hmm == 21 || hmm == 22 || hmm == 23 || hmm == 24)
  {
    return 1;
  }
  if (hmm == 25 || hmm == 27 || hmm == 28 || hmm == 30 || hmm == 31 || hmm == 33)
  {
    return 4;
  }
  if (hmm == 26 || hmm == 29 || hmm == 32)
  {
    return 6;
  }
  return 0;
}
/*
   Function Name      : calcsize
   Input              : hmm
   Output             : i
   Logic              : Returns the no. of neighbours of a node using the a value(hmm) mapped for the node
   Example Call       : calcSize(2)
*/
int calcsize(int hmm)
{
  int ind = 0;
  while (beer[hmm][ind] != '0')   /*Calculates size by detecting the position when '0' is encountered*/
    ind++;
  return ind;
}
/*
   Function Name      : mapp
   Input              : jk
   Output             : Mapped value of jk
   Logic              : Returns a mapped value for a character by modifying its ASCII value
   Example Call       : mapp(b)
*/
int mapp(int jk)
{
  if (jk >= 97)         /*for a-z */
    return jk - 97;
  else                  /*for A-Z*/
    return jk - 41;
}
/*
   Function Name      : mapp2
   Input              : int jk
   Output             : jk
   Logic              : It returns the ASCII value of input
   Example Call       : mapp2(c)
*/
int mapp2(int jk)
{
  return jk;
}
/*
   Function Name      : findPath
   Input              : char s, char dest
   Output             : Returns 0 or 1 (it actually works on bws, source and path)
   Logic              : It creates the path from one node(s) to another node(dest) using Breadth first search traversal of graphs(implemented through arrays)
   Example Call       : findPath(a,X)
*/
int findPath(char s, char dest)
{
  int bwsIndex = 0, ind = 0;
  int sourceIndex = 0;
  int flag = 0;
  if (s == dest)          /*If the source and destination are same we return 0*/
  {
    path[0] = s;
    len = 0;
    return 0;
  }
  bws[ind] = dest;
  source[sourceIndex] = 0;
  while (flag == 0)                            /*runs the loop till source(since we are backtracing) is not found*/
  {
    mml = bws[bwsIndex];
    int y = 0;
    for (y = 0; y < calcsize(mapp(mml)); y++)   /* used to access all the neighbours of mml*/
    {
      int x1 = 0, avai = 1;
      for (x1; x1 <= ind; x1++)
      {
        if (beer[mapp(mml)][y] == bws[x1])     /*Checks if the accessed neighbour already exists in bws*/
        {

          avai = 0;
          break;
        }
      }
      if (avai == 1)                          /*Adds the neighbour to bws if it does not already exist in it*/
      {
        ind++; sourceIndex++;
        bws[ind] = beer[mapp(mml)][y];
        source[sourceIndex] = bws[bwsIndex];
        if (bws[ind] == s)                    /*If source (since we are backtracing) is achieved, we break from the loop*/
        {
          flag = 1;
          break;
        }
      }
    }
    bwsIndex++;
  }
  path[0] = bws[ind];
  path[1] = source[sourceIndex];
  int tien, pathIndex = 1;
  for (tien = sourceIndex - 1; tien >= 0; tien--)   /*path is traced from the source to destination using bws and source*/
  { /*and storing it in path*/
    mml = path[pathIndex];
    int y = 0;
    for (y = 0; y < calcsize(mapp(mml)); y++)
    {
      if (beer[mapp(mml)][y] == source[tien])
      {
        pathIndex++;
        path[pathIndex] = source[tien];
        break;
      }
    }
  }
  len = pathIndex;                /*len stores the size(that needs to be accessed) of the path aaray */
  return 1;
}
/*
   Function Name      : deterPath
   Input              : char start, int destNode
   Output             : None
   Logic              : It finds the most optimum path from our current position to the required MNP
   Example Call       : deterPath(a,29)
*/
void deterPath(char start, int destNode)
{
  char arr1[48];
  char arr2[48];
  char arr3[48];
  char arr4[48];
  char arr5[48];
  char arr6[48];
  int siz = calcSizeNe(destNode);    /*siz stores the no. of nodes that can approach the MNP*/
  int index, ind = 0;
  int temp, small, leastPath;
  while (ind < siz)                /*runs for all the nodes that can approach the MNP*/
  {
    switch (ind)                   /*used to find path from start to each node that can access the MNP, save them seperately, and compare the length*/
    {
      case 0:
        {
          findPath(start, neigh[destNode - 1][ind]);
          for (index = 0; index <= len; index++)

            arr1[index] = path[index];

          small = index;
          leastPath = 1;
          break;
        }
      case 1:
        {
          findPath(start, neigh[destNode - 1][ind]);
          for (index = 0; index <= len; index++)
            arr2[index] = path[index];
          if (small > index)
          {
            small = index;
            leastPath = 2;
          }
          break;
        }
      case 2:
        {
          findPath(start, neigh[destNode - 1][ind]);
          for (index = 0; index <= len; index++)
            arr3[index] = path[index];
          if (small > index)
          {
            small = index;
            leastPath = 3;
          }
          break;
        }
      case 3:
        {
          findPath(start, neigh[destNode - 1][ind]);
          for (index = 0; index <= len; index++)
            arr4[index] = path[index];
          if (small > index)
          {
            small = index;
            leastPath = 4;
          }
          break;
        }
      case 4:
        {
          findPath(start, neigh[destNode - 1][ind]);
          for (index = 0; index <= len; index++)
            arr5[index] = path[index];
          if (small > index)
          {
            small = index;
            leastPath = 5;
          }
          break;
        }
      case 5:                        /*there are 6 cases since the maximum nodes can be only 6 for any MNP*/
        {
          findPath(start, neigh[destNode - 1][ind]);
          for (index = 0; index <= len; index++)
            arr6[index] = path[index];
          if (small > index)
          {
            small = index;
            leastPath = 6;
          }
          break;
        }
    }
    ind++;
  }
  switch (leastPath)     /*Stores the smallest path in path array*/
  {
    case 1:
      {
        for (temp = 0; temp <= small; temp++)
          path[temp] = arr1[temp];
        break;
      }
    case 2:
      {
        for (temp = 0; temp <= small; temp++)
          path[temp] = arr2[temp];
        break;
      }
    case 3:
      {
        for (temp = 0; temp <= small; temp++)
          path[temp] = arr3[temp];
        break;
      }
    case 4:
      {
        for (temp = 0; temp <= small; temp++)
          path[temp] = arr4[temp];
        break;
      }
    case 5:
      {
        for (temp = 0; temp <= small; temp++)
          path[temp] = arr5[temp];
        break;
      }
    case 6:
      {
        for (temp = 0; temp <= small; temp++)
          path[temp] = arr6[temp];
        break;
      }
  }
  len = small;   /*stores the size of path(that requires to be accessed) in len */
}
/*
   Function Name      : deleteNeighbour
   Input              : char sa, char re
   Output             : None
   Logic              : deletes the nodes from each other's neighbours when an obstacle is detected in between them to avoid the path being considered again
   Example Call       : deleteNeighbour(b,c)
*/
void deleteNeighbour(char sa, char re)
{
  int sizesa = calcsize(mapp(sa));   /* Stores the no. of neighbours of sa*/
  int ind = 0;
  for (ind; ind < sizesa; ind++)     /*finds position of re in neighbours of sa(in the beer 2-D array)*/
  {
    if (beer[mapp(sa)][ind] == re)
    {
      break;
    }
  }
  beer[mapp(sa)][ind] = beer[mapp(sa)][sizesa - 1]; /*swap re with the last neighbour of sa*/
  beer[mapp(sa)][sizesa - 1] = '0';                 /*replace re with '0'. This is in accordance to calcsize function */

  int sizere = calcsize(mapp(re));    /*Repeat the procedure for re*/
  ind = 0;
  for (ind; ind < sizere; ind++)
  {
    if (beer[mapp(re)][ind] == sa)
    {
      break;
    }
  }
  beer[mapp(re)][ind] = beer[mapp(re)][sizere - 1];
  beer[mapp(re)][sizere - 1] = '0';
}
/*
   Function Name      : Stop
   Input              : None
   Output             : None
   Logic              : Stops both the motors
   Example Call       : Stop()
*/
void Stop()
{
  PORTA = 0B00000000;
}

/*
   Function Name      : Forw
   Input              : None
   Output             : None
   Logic              : Starts both the motors in forward direction
   Example Call       : Forw()
*/
void Forw()
{
  PORTA = 0B00000110;
}

/*
   Function Name      : Back
   Input              : None
   Output             : None
   Logic              : Starts both the motors in backward direction
   Example Call       : Back()
*/
void Back()
{
  PORTA = 0B00001001;
}

/*
   Function Name      : Left
   Input              : None
   Output             : None
   Logic              : Starts right motor in forward direction and left motor in backward direction
   Example Call       : Left()
*/
void Left()
{
  PORTA = 0B00000101;
}

/*
   Function Name      : Right
   Input              : None
   Output             : None
   Logic              : Starts left motor in forward direction and right motor in backward direction
   Example Call       : Right()
*/
void Right()
{
  PORTA = 0B00001010;
}
/*
   Function Name      : Line_Follow_Forw
   Input              : None
   Output             : None
   Logic              : Follows the line
   Example Call       : Line_Follow_Forw()
*/
int Line_Follow_Forw()
{
  //Update sensor values
  ls = analogRead(3);
  ms = analogRead(2);
  rs = analogRead(1);
  if (verify == 1)
  {
    if (analogRead(A6) <= 350)        //Proximity sensor value
    {
      obstacle = 1;
      Stop();
      return 2;
    }
  }
  if ((ls <= Thresh) && (ms > Thresh) && (rs <= Thresh))
    Forw();
  if ((ls > Thresh) && (ms <= Thresh) && (rs <= Thresh))
    Left();
  else if ((ls <= Thresh) && (ms <= Thresh) && (rs > Thresh))
    Right();
  else
    Forw();
  //return 1 if a node is detected
  if ((ms + rs + ls) > 800)
  {
    Forw_Cm(8);                       //Allign the bot on node
    return 1;
  }
  else
    return 0;
}
/*
   Function Name      : Line_Follow_Back
   Input              : None
   Output             : None
   Logic              : Follows the line
   Example Call       : Line_Follow_Back()
*/
int Line_Follow_Back()
{
  //Update sensor values
  ls = analogRead(3);
  ms = analogRead(2);
  rs = analogRead(1);
  if ((ls <= Thresh) && (ms > Thresh) && (rs <= Thresh))
    Back();
  if ((ls > Thresh) && (ms <= Thresh) && (rs <= Thresh))
    Left();
  else if ((ls <= Thresh) && (ms <= Thresh) && (rs > Thresh))
    Right();
  else
    Back();
  //return 1 if a node is detected
  if ((ms + rs + ls) > 800)
  {
    Forw_Cm(8);                       //Allign the bot on node
    return 1;
  }
  else
    return 0;
}
/*
   Function Name      : Front
   Input              : a
   Output             : None
   Logic              : Moves 'a' cells forward
   Example Call       : Front(2)
*/
void Front(int a = 1)
{
  int Node_Check = 0;
  //Follow the line till 'a' nodes
  while (a >= 1)
  {
    Node_Check = Line_Follow_Forw();
    a--;
    while (Node_Check == 0)                           //while no node detected
    {
      Node_Check = Line_Follow_Forw();                //Follow Line
      if (Node_Check == 2)
        break;
    }
  }
}
/*
   Function Name      : Rev
   Input              : a
   Output             : None
   Logic              : Moves 'a' cells forward
   Example Call       : Rev(2)
*/
void Rev(int a = 1)
{
  int Node_Check = 0;
  //Follow the line till 'a' nodes
  while (a >= 1)
  {
    Node_Check = Line_Follow_Back();
    a--;
    while (Node_Check == 0)                           //while no node detected
    {
      Node_Check = Line_Follow_Back();                //Follow Line
    }
  }
}
/*
   Function Name      : Count_Right
   Input              : None
   Output             : None
   Logic              : Interrupt Service Routine for Right Position Encoder
   Example Call       : Count_Right()
*/
void Count_Right()
{
  ShaftCountRight++;
}
/*
   Function Name      : Count_Left
   Input              : None
   Output             : None
   Logic              : Interrupt Service Routine for Left Position Encoder
   Example Call       : Count_Left()
*/
void Count_Left()
{
  ShaftCountLeft++;
}
/*
   Function Name      : Forw_Cm
   Input              : Distance
   Output             : None
   Logic              : Follows the line in forward direction for 'distance' cm
   Example Cal        : Forw_Cm(15)
*/
int Forw_Cm(int Distance)
{
  // Mapping Function to calculate Shaft Counts
  int ReqdShaftCount = (3) / 0.5338 ;
  ShaftCountRight = 0;
  verify = 0;
  while (ShaftCountRight < ReqdShaftCount)
    Forw();
  Stop();
  verify = 1;
  ReqdShaftCount = (Distance - 4) / 0.5338 ;
  ShaftCountRight = 0;
  verify = 0;
  while (ShaftCountRight < ReqdShaftCount)
    Line_Follow_Forw();
  Stop();
  verify = 1;
}

/*
   Function Name      : Back_Cm
   Input              : Distance
   Output             : None
   Logic              : Follows the line in backward direction for 'distance' cm
   Example Call       : Back_Cm(15)
*/
void Back_Cm(int Distance)
{
  // Mapping Function to calculate Shaft Counts
  int ReqdShaftCount = Distance / 0.5338 ;
  ShaftCountRight = 0;
  while (ShaftCountRight < ReqdShaftCount)
    Line_Follow_Back();
  Stop();
}
/*
   Function Name      : Left_D
   Input              : Degrees
   Output             : None
   Logic              : Rotates the Firebird to left by 'degrees'
   Example Call       : Left_D(60)
*/
void Left_D(unsigned int Degrees)
{
  int ReqdShaftCount = (Degrees - 10) / 4.090 ;                         //Rotate for degree-10
  ShaftCountRight = 0;
  while (ShaftCountRight < ReqdShaftCount)
    Left();
  ReqdShaftCount = (20) / 4.090 ;
  ShaftCountRight = 0;
  ms = analogRead(2);                                                   //Update sensor values
  while ((ms < Thresh) && (ShaftCountRight < ReqdShaftCount))           //Rotate while no black line detected
  {
    ms = analogRead(2);
    Left();
  }
  Stop();
}
/*
   Function Name      : Right_D
   Input              : Degrees
   Output             : None
   Logic              : Rotates the Firebird to right by 'degrees'
   Example Call       : Right_D(60)
*/
void Right_D(unsigned int Degrees)
{
  int ReqdShaftCount = (Degrees - 10) / 4.090 ;                         //Rotate for degree-10
  ShaftCountRight = 0;
  while (ShaftCountRight < ReqdShaftCount)
    Right();
  ReqdShaftCount = (20) / 4.090 ;
  ShaftCountRight = 0;
  ms = analogRead(2);                                                   //Update sensor values
  while ((ms < Thresh) && (ShaftCountRight < ReqdShaftCount))           //Rotate while no black line detected
  {
    ms = analogRead(2);
    Right();
  }
  Stop();
}
/*
   Function Name      : face
   Input              : f2
   Output             : None
   Logic              : Rotates the Firebird by Degrees according to direction
   Example Call       : face('e')
*/
void face(char f2)
{ //f2 is the direction to be faced
  //f1 is the direction currently facing
  //e-east
  //w-west
  //s-south west
  //S-South East
  //n-north west
  //N-north East
  if (f2 == 'n')
  {
    if (f1 == 'N')
      Left_D(60);
    else if (f1 == 'e')
      Left_D(120);
    else if (f1 == 'S')
      Left_D(180);
    else if (f1 == 's')
      Right_D(120);
    else if (f1 == 'w')
      Right_D(60);
  }
  else if (f2 == 'N')
  {
    if (f1 == 'e')
      Left_D(60);
    else if (f1 == 'S')
      Left_D(120);
    else if (f1 == 's')
      Left_D(180);
    else if (f1 == 'w')
      Right_D(120);
    else if (f1 == 'n')
      Right_D(60);
  }
  else if (f2 == 'e')
  {
    if (f1 == 'S')
      Left_D(60);
    else if (f1 == 's')
      Left_D(120);
    else if (f1 == 'w')
      Left_D(180);
    else if (f1 == 'n')
      Right_D(120);
    else if (f1 == 'N')
      Right_D(60);
  }
  else if (f2 == 'S')
  {
    if (f1 == 's')
      Left_D(60);
    else if (f1 == 'w')
      Left_D(120);
    else if (f1 == 'n')
      Left_D(180);
    else if (f1 == 'N')
      Right_D(120);
    else if (f1 == 'e')
      Right_D(60);
  }
  else if (f2 == 's')
  {
    if (f1 == 'w')
      Left_D(60);
    else if (f1 == 'n')
      Left_D(120);
    else if (f1 == 'N')
      Left_D(180);
    else if (f1 == 'e')
      Right_D(120);
    else if (f1 == 'S')
      Right_D(60);
  }
  else if (f2 == 'w')
  {
    if (f1 == 'n')
      Left_D(60);
    else if (f1 == 'N')
      Left_D(120);
    else if (f1 == 'e')
      Left_D(180);
    else if (f1 == 'S')
      Right_D(120);
    else if (f1 == 's')
      Right_D(60);
  }
  f1 = f2;                                            //Dead Reckoning used to update current values
}
/*
   Function Name      : Buzzer
   Input              : seconds
   Output             : None
   Logic              : Plays the Buzzer for 'seconds'
   Example Call       : Buzzer(1)
*/
void Buzzer(int seconds)
{
  digitalWrite(34, HIGH);
  delay(seconds);
  digitalWrite(34, LOW);
}
/*
   Function Name      : allign
   Input              : None
   Output             : None
   Logic              : Alligns the bot in particular direction according to current config
   Example Call       : allign()
*/
void allign()
{
  if ((Next_Pos == 'c') &&  (Curr_Pos == 'b') && (clockwise == 1))
    f1 = 'N';
  else if ((Next_Pos == 'c')  &&  (Curr_Pos == 'd') && (clockwise == 0))
    f1 = 's';
  else if ((Next_Pos == 'g')  &&  (Curr_Pos == 'f') && (clockwise == 1))
    f1 = 'e';
  else if ((Next_Pos == 'g')  &&  (Curr_Pos == 'h') && (clockwise == 0))
    f1 = 'w';
  else if ((Next_Pos == 'k')  &&  (Curr_Pos == 'j') && (clockwise == 1))
    f1 = 'S';
  else if ((Next_Pos == 'k')  &&  (Curr_Pos == 'l') && (clockwise == 0))
    f1 = 'n';
  else if ((Next_Pos == 'o')  &&  (Curr_Pos == 'n') && (clockwise == 1))
    f1 = 's';
  else if ((Next_Pos == 'o')  &&  (Curr_Pos == 'p') && (clockwise == 0))
    f1 = 'N';
  else if ((Next_Pos == 's')  &&  (Curr_Pos == 'r') && (clockwise == 1))
    f1 = 'w';
  else if ((Next_Pos == 's')  &&  (Curr_Pos == 't') && (clockwise == 0))
    f1 = 'e';
  else if ((Next_Pos == 'w')  &&  (Curr_Pos == 'v') && (clockwise == 1))
    f1 = 'n';
  else if ((Next_Pos == 'w')  &&  (Curr_Pos == 'x') && (clockwise == 0))
    f1 = 'S';
}
/*
   Function Name      : set
   Input              : None
   Output             : None
   Logic              : Sets the firebird in desired direction
   Example Call       : set()
*/
void set()
{
  if ((Curr_Pos == 's') && (Next_Pos == 'r'))                               //Decide the direction to be faced on the basis of Current position and next position
  {
    face('e');
    clockwise = 0;                                                          //Clockwise is used to store the current orientation of the bot on the outer circle
  }                                                                         //face is used to update the virtual compass
  else if ((Curr_Pos == 's') && (Next_Pos == 't'))
  {
    face('w');
    clockwise = 1;
  }
  else if ((Curr_Pos == 'w') && (Next_Pos == 'v'))
  {
    face('S');
    clockwise = 0;
  }
  else if ((Curr_Pos == 'w') && (Next_Pos == 'x'))
  {
    face('n');
    clockwise = 1;
  }
  else if ((Curr_Pos == 'o') && (Next_Pos == 'n'))
  {
    face('N');
    clockwise = 0;
  }
  else if ((Curr_Pos == 'o') && (Next_Pos == 'p'))
  {
    face('s');
    clockwise = 1;
  }
  else if ((Curr_Pos == 'k') && (Next_Pos == 'j'))
  {
    face('n');
    clockwise = 0;
  }
  else if ((Curr_Pos == 'k') && (Next_Pos == 'l'))
  {
    face('S');
    clockwise = 1;
  }
  else if ((Curr_Pos == 'g') && (Next_Pos == 'f'))
  {
    face('w');
    clockwise = 0;
  }
  else if ((Curr_Pos == 'g') && (Next_Pos == 'h'))
  {
    face('e');
    clockwise = 1;
  }
  else if ((Curr_Pos == 'c') && (Next_Pos == 'b'))
  {
    face('s');
    clockwise = 0;
  }
  else if ((Curr_Pos == 'a') && (Next_Pos == 'x'))
  {
    if (clockwise == 1)
    {
      Left_D(180);
      clockwise = 0;
    }
  }
  else if ((Curr_Pos == 'x') && (Next_Pos == 'a'))
  {
    if (clockwise == 0)
    {
      Right_D(180);
      clockwise = 1;
    }
  }
  else if ((Curr_Pos == 'c') && (Next_Pos == 'd'))
  {
    face('N');
    clockwise = 1;
  }
  else if ((Next_Pos > Curr_Pos) && (mapp2(Curr_Pos) >= 97) && (mapp2(Next_Pos) >= 97) && (clockwise == 0))         //when on outer circle and moving anti-clockwise
  {
    Right_D(180);
    clockwise = 1;
  }
  else if ((Next_Pos < Curr_Pos) && (mapp2(Curr_Pos) >= 97) && (mapp2(Next_Pos) >= 97) && (clockwise == 1))         //when on outer circle and moving clockwise
  {
    Left_D(180);
    clockwise = 0;
  }

  //When inside the circle
  else if ((Curr_Pos == 'G') && ( Next_Pos == 'F'))
    face('e');
  else if ((Curr_Pos == 'G') && ( Next_Pos == 'c'))
    face('w');
  else if ((Curr_Pos == 'G' ) && ( Next_Pos == 'B'))
    face('s');
  else if ((Curr_Pos == 'B' ) && ( Next_Pos == 'c'))
    face('n');
  else if ((Curr_Pos == 'B' ) && ( Next_Pos == 'C'))
    face('S');
  else if ((Curr_Pos == 'B' ) && ( Next_Pos == 'G'))
    face('N');
  else if ((Curr_Pos == 'C' ) && ( Next_Pos == 'B'))
    face('n');
  else if ((Curr_Pos == 'C' ) && ( Next_Pos == 'D'))
    face('e');
  else if ((Curr_Pos == 'C' ) && ( Next_Pos == 'A'))
    face('s');
  else if ((Curr_Pos == 'C' ) && ( Next_Pos == 'U'))
    face('S');
  else if ((Curr_Pos == 'D' ) && ( Next_Pos == 'C'))
    face('w');
  else if ((Curr_Pos == 'D' ) && ( Next_Pos == 'E'))
    face('N');
  else if ((Curr_Pos == 'E' ) && ( Next_Pos == 'D'))
    face('s');
  else if ((Curr_Pos == 'E' ) && ( Next_Pos == 'F'))
    face('n');
  else if ((Curr_Pos == 'F' ) && ( Next_Pos == 'E'))
    face('S');
  else if ((Curr_Pos == 'F' ) && ( Next_Pos == 'G'))
    face('w');
  else if ((Curr_Pos == 'F' ) && ( Next_Pos == 'H'))
    face('N');
  else if ((Curr_Pos == 'F' ) && ( Next_Pos == 'O'))
    face('e');
  else if ((Curr_Pos == 'c') && (Next_Pos == 'G'))
    face('e');
  else if ((Curr_Pos == 'c') && (Next_Pos == 'B'))
    face('S');
  else if ((Curr_Pos == 'w') && (Next_Pos == 'A'))
    face('N');
  else if ((Curr_Pos == 'w') && (Next_Pos == 'X'))
    face('e');
  else if ((Curr_Pos == 'A') && (Next_Pos == 'w'))
    face('s');
  else if ((Curr_Pos == 'A') && (Next_Pos == 'C'))
    face('N');
  else if ((Curr_Pos == 'X') && (Next_Pos == 'w'))
    face('w');
  else if ((Curr_Pos == 'X') && (Next_Pos == 'U'))
    face('e');
  else if ((Curr_Pos == 's') && (Next_Pos == 'T'))
    face('n');
  else if ((Curr_Pos == 's') && (Next_Pos == 'S'))
    face('N');
  else if ((Curr_Pos == 'o') && (Next_Pos == 'P'))
    face('n');
  else if ((Curr_Pos == 'o') && (Next_Pos == 'Q'))
    face('w');
  else if ((Curr_Pos == 'P') && (Next_Pos == 'o'))
    face('S');
  else if ((Curr_Pos == 'P') && (Next_Pos == 'L'))
    face('n');
  else if ((Curr_Pos == 'Q') && (Next_Pos == 'o'))
    face('e');
  else if ((Curr_Pos == 'Q') && (Next_Pos == 'R'))
    face('w');
  else if ((Curr_Pos == 'k') && (Next_Pos == 'J'))
    face('w');
  else if ((Curr_Pos == 'k') && (Next_Pos == 'K'))                                  //Mapping the map using Dead Reckoning
    face('s');
  else if ((Curr_Pos == 'g') && (Next_Pos == 'H'))
    face('s');
  else if ((Curr_Pos == 'g') && (Next_Pos == 'I'))
    face('S');
  else if ((Curr_Pos == 'H') && (Next_Pos == 'g'))
    face('N');
  else if ((Curr_Pos == 'H') && (Next_Pos == 'F'))
    face('s');
  else if ((Curr_Pos == 'I') && (Next_Pos == 'g'))
    face('n');
  else if ((Curr_Pos == 'I') && (Next_Pos == 'O'))
    face('S');
  else if ((Curr_Pos == 'V') && (Next_Pos == 'W'))
    face('e');
  else if ((Curr_Pos == 'V') && (Next_Pos == 'U'))
    face('s');
  else if ((Curr_Pos == 'W') && (Next_Pos == 'R'))
    face('S');
  else if ((Curr_Pos == 'W') && (Next_Pos == 'V'))
    face('w');
  else if ((Curr_Pos == 'R') && (Next_Pos == 'W'))
    face('n');
  else if ((Curr_Pos == 'R') && (Next_Pos == 'L'))
    face('N');
  else if ((Curr_Pos == 'R') && (Next_Pos == 'Q'))
    face('e');
  else if ((Curr_Pos == 'R') && (Next_Pos == 'S'))
    face('s');
  else if ((Curr_Pos == 'S') && (Next_Pos == 's'))
    face('s');
  else if ((Curr_Pos == 'S') && (Next_Pos == 'T'))
    face('w');
  else if ((Curr_Pos == 'S') && (Next_Pos == 'R'))
    face('N');
  else if ((Curr_Pos == 'T') && (Next_Pos == 'S'))
    face('e');
  else if ((Curr_Pos == 'T') && (Next_Pos == 's'))
    face('S');
  else if ((Curr_Pos == 'T') && (Next_Pos == 'U'))
    face('n');
  else if ((Curr_Pos == 'U') && (Next_Pos == 'X'))
    face('w');
  else if ((Curr_Pos == 'U') && (Next_Pos == 'C'))
    face('n');
  else if ((Curr_Pos == 'U') && (Next_Pos == 'V'))
    face('N');
  else if ((Curr_Pos == 'U') && (Next_Pos == 'T'))
    face('S');
  else if ((Curr_Pos == 'O') && (Next_Pos == 'I'))
    face('n');
  else if ((Curr_Pos == 'O') && (Next_Pos == 'J'))
    face('e');
  else if ((Curr_Pos == 'O') && (Next_Pos == 'F'))
    face('w');
  else if ((Curr_Pos == 'O') && (Next_Pos == 'N'))
    face('s');
  else if ((Curr_Pos == 'J') && (Next_Pos == 'O'))
    face('w');
  else if ((Curr_Pos == 'J') && (Next_Pos == 'k'))
    face('e');
  else if ((Curr_Pos == 'J') && (Next_Pos == 'K'))
    face('S');
  else if ((Curr_Pos == 'K') && (Next_Pos == 'J'))
    face('n');
  else if ((Curr_Pos == 'K') && (Next_Pos == 'k'))
    face('N');
  else if ((Curr_Pos == 'K') && (Next_Pos == 'L'))
    face('s');
  else if ((Curr_Pos == 'L') && (Next_Pos == 'K'))
    face('N');
  else if ((Curr_Pos == 'L') && (Next_Pos == 'P'))
    face('S');
  else if ((Curr_Pos == 'L') && (Next_Pos == 'M'))
    face('w');
  else if ((Curr_Pos == 'L') && (Next_Pos == 'R'))
    face('s');
  else if ((Curr_Pos == 'M') && (Next_Pos == 'L'))
    face('e');
  else if ((Curr_Pos == 'M') && (Next_Pos == 'N'))
    face('n');
  else if ((Curr_Pos == 'N') && (Next_Pos == 'M'))
    face('S');
  else if ((Curr_Pos == 'N') && (Next_Pos == 'O'))
    face('N');
}
/*
   Function Name      : Traverse
   Input              : None
   Output             : None
   Logic              : Takes the bot to the desired location and updates its Current location
   Example Call       : Traverse()
*/
void Traverse()
{
  weed = 0;
  Prev_Pos = path[0];
  Curr_Pos = path[0];                                   //Update podition parameters
  Next_Pos = path[1];
  ch = Next_Pos;
  for (weed = 0; weed < len - 1; weed++)                //loop to traverse the path from initial pos to destination position
  {
    Prev_Pos = Curr_Pos;
    Curr_Pos = path[weed];
    Next_Pos = path[weed + 1];
    allign();
    set();
    if (Curr_Pos != Next_Pos)                           //only if next position is not equal to current position
    {
      Front();                                          //Move ahead by one node
    }
    if (obstacle == 1)                                  //obstacle detected
    {
      Rev();
      obstacle = 0;
      deleteNeighbour(Next_Pos, Curr_Pos);              //Determine new path
      deterPath(Curr_Pos, reach);
      Traverse();
      return 0;
    }
    allign();
  }
  return 1;
}
/*
   Function Name      : Play
   Input              : None
   Output             : None
   Logic              : Follows the path according to given mnps
   Example Call       : Play()
*/
int Play()
{
  ch = a;
  clockwise = 1;
  int mnp_size = sizeof(mnp);
  for (pos; pos < mnp_size; pos++)                    //Traverse the MNP array
  {
    reach = mnp[pos];
    deterPath(ch, mnp[pos]);
    Traverse();
    if (mnp[pos] == mnp_size - 1)                     //Long Buzzer
      Buzzer(4500);
    Buzzer(500);                                      //Short Buzzer
    delay(500);
    ch = path[len - 1];                               //Update Current position
  }
  return 0;
}
/*
   Function Name      : Setup
   Input              : None
   Output             : None
   Logic              : Initializes all the required pins and ports.
   Example Call       : setup()
*/
void setup() {
  // put your setup code here, to run once:
  // PORTS Initiallization
  DDRA = 0x0F;
  DDRL = DDRL | 0x18;
  PORTL = PORTL | 0x18;
  DDRE  = DDRE & 0xEF;
  PORTE = PORTE | 0x10;
  DDRE  = DDRE & 0xDF;
  PORTE = PORTE | 0x20;
  pinMode(A6, INPUT);                                     //A6 is the proximity sensor in front of the bot
  pinMode(3, INPUT);                                      //1,2,3 arethe White line sensors
  pinMode(2, INPUT);
  pinMode(1, INPUT);
  pinMode(34, OUTPUT);                                    //Buzzer Pin
  pinMode(45, OUTPUT);                                    //PWM inputs of motor drivers
  pinMode(46, OUTPUT);
  analogWrite(45, 225);
  // Interrupts Initialization
  attachInterrupt(digitalPinToInterrupt(2), Count_Left, FALLING);
  attachInterrupt(digitalPinToInterrupt(3), Count_Right, FALLING);
  delay(1000);

  Play();                                                             //Let the Game begin :)
}
/*
   Function Name      : loop
   Input              : None
   Output             : None
   Logic              : None
   Example Call       : loop()
*/
void loop() {
  // put your main code here, to run repeatedly:
}

