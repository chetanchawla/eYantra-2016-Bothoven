
/******************************************************************************************************************************
   Code                : Slave Driver Code
   Team Id             : BV#901
   Author List         : Divyansh Malhotra, Chetan Chawla, Harshil Bansal,Ishani Janveja
   Filename            : eYRC-BV#901
   Theme               : Bothoven
   Functions           : calcSizeNe,calcsize,mapp,mapp2,findPath,deterPath,deleteNeighbour,Stop,Forw,Back,Left,Right,face,
                         Line_Follow_Forw,Line_Follow_Back,Front,Count_Right,Count_Left,Forw_Cm,Back_Cm,Traverse,Buzzer,
                         set,allign,Right_D,Left_D, Rev, Play, giveMNP, sharp_distance, striking_left, striking_right, 
                         striking_middle, hit, meraSerial, chotaBuzzer, getSlave, insertNeighbour, again_serial
   Global Variables    : ShaftCountRight, ShaftCountLeft, Thresh, ls, ms, rs, mnp, mml, a,b....x,A,B,....X, bws, source,
                         path, neigh, beer, temp, ch, f1, len, reach, Curr_Pos, Next_Pos, Prev_Pos, clockwise, obstacle,
                         verify, weed, pos,servoLeft, servoRight, pinNo, number, number1, DistanceDash, Distance, mnps, 
                         sizeMNP, indS, counter, blast, same, flagg, 
*                                                                                                                             *
*******************************************************************************************************************************/
#include<Servo.h>
Servo myservo1;                                                            /*myservo1 is the servo object referring to rotating servo*/
Servo myservo2;                                                             /*myservo2 is the servo object referring to striking servo*/
int servoLeft = 0, servoRight = 180, pinNo = A11, number, number1, modelNo, DistanceDash; 
float sDistance;
void meraSerial();
int mnp[] = {3, 16, 15, 20, 13, 2, 2, 2, 7, 3, 2, 16, 19, 7, 14};
//int mnp1[] = {16, 0, 20, 3, 14, 11, 4, 9, 1, 1};
//char mnp[33];
int arr[2];
int masterDrive[33][2];
int slaveDrive[33][2];
int indS, indM;
int slave_size;
//to be filled according to table
int notesToMNP[21][2] = {
  /*{32, 33}, //A6 0
  {18, 0}, //B6  1
  {2, 0}, //C6  2  //yah 2,28 tha
  {8, 0}, //D6  3
  {20, 0}, //E6  4
  {0, 0}, //F6 5  //yaha 9 krdo
  {0, 0}, //G6  6
  {0, 0}, //A7 7
  {0, 0}, //B7 8
  {23, 0}, //C7  9
  {0, 0}, //D7  10
  {30, 0}, //E7  11
  {6, 0}, //F7  12
  {4, 0}, //G7 13
  {26, 0}, //A8  14
  {0, 0}, //B8  15
  {22, 0}, //C8 16
  {0, 0}, //D8 17
  {0, 0}, //E8  18
  {0, 0}, //F8  19
  {12, 0}, //G8  20*/
  {0, 0}, //A6 0
  {0, 0}, //B6  1
  {0, 0}, //C6  2  //yah 2,28 tha
  {8, 13}, //D6  3
  {0, 0}, //E6  4
  {9, 0}, //F6 5  //yaha 9 krdo
  {33, 0}, //G6  6
  {32, 0}, //A7 7
  {0, 0}, //B7 8
  {1, 0}, //C7  9
  {0, 0}, //D7  10
  {19, 0}, //E7  11
  {21, 0}, //F7  12
  {4, 0}, //G7 13
  {16, 20}, //A8  14
  {5, 29}, //B8  15
  {22, 0}, //C8 16
  {0, 0}, //D8 17
  {18, 0}, //E8  18
  {26, 1}, //F8  19
  {10, 0}, //G8  20
};
int mnps;    //size of arr
/*
   Function Name      : giveMNP
   Input              : note
   Output             : None
   Logic              : Set value of mnps as 0,1 or 2 and stores contents of notesToMNP 2D array into arr
   Example Call       : giveMNP(1)
*/
void giveMNP(int note)
{
  arr[0] = notesToMNP[note][0];
  arr[1] = notesToMNP[note][1];
  if (arr[0] == 0)
    mnps = 0;
  else if (arr[1] == 0)
    mnps = 1;
  else
    mnps = 2;
}

char mml;
int sizeMNP=15;
//int sizeMNP;
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
char ch = 'm', f1 = 'n';
int len = 0, reach = 1;
int ShaftCountRight = 0, ShaftCountLeft = 0;                               /*ISR increaments*/
int Thresh = 48; //wannachange 48                                          /* White Line Sensor Threshhold Value*/
int ls = 0, ms = 0, rs = 0;                                                /* White Line Sensor Values*/
int Curr_Pos, Next_Pos, Prev_Pos;
int clockwise = 1, obstacle = 0, verify = 1, weed = 0, pos = 0;
int counter = 0;                                                           /*sequencing of notes to be played*/
int blast;                                                                 /*For obstacle avoidance when master is at its note- is used as a flag when incorrect data is recieved by serial communication*/
int same;                                                                  /*used for striking at same or different orientation  when current and next postion is same*/
int flagg;                                                                 /*a dummy value of flag in traverse. Used when wrong data is recieved serially*/
int sarkar=2;
int moonfli;
int tenWala=10;                                                               /*used as initial handshaking bit for serial comm.*/
char its_pos;
/*
   Function Name      : sharp_distance
   Input              : None
   Output             : Distance from the sharp sensor to an object(in cm)
   Logic              : Returns the distance of note from sharp sensor for correct identification of a note.
   Example Call       : sharp_distance()
*/
float sharp_distance()
{
  float val = analogRead(pinNo);
  float dist_cm = 10.00 * ((1.00 / ((0.001240875 * (float)val) + 0.005)) - 0.42) / 100.00;
  dist_cm = (dist_cm * 100);
  return dist_cm;
}

/*
   Function Name      : striking_left
   Input              : None
   Output             : None
   Logic              : Sets the rotating servo to leftmost position and checks for note in a range while moving rightwards, hits it if distance <=23cm and brings back servos to original positions
   Example Call       : striking_left()
*/
void striking_left()
{
  //myservo1.attach(11);
  //myservo2.attach(12);
  myservo1.write(servoLeft);
  same = 1;
  delay(1000);
  for (int i = servoLeft; i < 80; i++)
  {
    myservo1.write(i);
    sDistance = sharp_distance();
    //Serial2.println(analogRead(pinNo));
    //Serial2.println(sDistance);
    if (sDistance < 13)
    {
      myservo1.write(i + 6);
      delay(500);
      myservo2.write(40);
      delay(500);
      myservo2.write(180);
      delay(500);
      myservo1.write(0);
      delay(500);
      //      myservo1.detach();
      //      myservo2.detach();
      break;
    }
    delay(100);

  }
}

/*
   Function Name      : striking_right
   Input              : None
   Output             : None
   Logic              : Sets the rotating servo to rightmost position and checks for note in a range while moving left, hits it if distance <=23cm and brings back servos to original positions
   Example Call       : striking_right()
*/
void striking_right()
{
  //myservo1.attach(11);
  //myservo2.attach(12);
  myservo1.write(servoRight);
  delay(1000);
  same = 0;
  for (int i = servoRight; i > 100; i--)
  {
    myservo1.write(i);
    sDistance = sharp_distance();
    if (sDistance < 13)
    {
      myservo1.write(i - 7);
      delay(500);
      myservo2.write(40);
      delay(500);
      myservo2.write(180);
      delay(500);
      myservo1.write(0);
      delay(500);
      //  myservo1.detach();
      //  myservo2.detach();
      break;
    }
    delay(50);
  }
}
/*
   Function Name      : striking_middle
   Input              : None
   Output             : None
   Logic              : Sets the rotating servo to slightly leftwards to middle position and checks for note in a range while moving right, hits it if distance <=23cm and brings back servos to original positions
   Example Call       : striking_middle()
*/
void striking_middle()
{
  //myservo1.attach(11);
  //myservo2.attach(12);
  same = 2;
  for (int i = 60; i < 120; i++)
  {
    myservo1.write(i);
    sDistance = sharp_distance();
    if (sDistance < 13)
    {
      myservo1.write(i + 5);
      delay(500);
      myservo2.write(40);
      delay(500);
      myservo2.write(180);
      delay(500);
      myservo1.write(0);
      delay(500);
      //myservo1.detach();
      //myservo2.detach();
      break;
    }
    delay(50);
  }
}


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
   Function Name      : chotaBuzzer
   Input              : seconds
   Output             : None
   Logic              : Plays the Buzzer for 'seconds' microseconds. 
   Example Call       : chotaBuzzer(250000)
*/
void chotaBuzzer(int seconds)
{
  digitalWrite(34, HIGH);
  delayMicroseconds(seconds);
  digitalWrite(34, LOW);
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
int findPath(char so, char dest)
{
  int bwsIndex = 0, ind = 0;
  int sourceIndex = 0;
  int flag = 0;
  if (so == dest)          /*If the source and destination are same we return 0*/
  {
    path[0] = so;
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
        //Serial2.println(bws[ind]);
        source[sourceIndex] = bws[bwsIndex];
        if (bws[ind] == so)                    /*If source (since we are backtracing) is achieved, we break from the loop*/
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
  //Serial2.println("Path Determined: ");
  //for(int i=0;i<=pathIndex;i++)
  //{
    //Serial2.println(path[i]);
    //}
    //hataya serial print
  if(path[pathIndex]!=dest)
  {
    return 2;
    }  
  len = pathIndex;                /*len stores the size(that needs to be accessed) of the path aaray */
  return 1;
}
/*int findPath(char s, char dest)
{
  int bwsIndex = 0, ind = 0;
  int sourceIndex = 0;
  int flag = 0;
  if (s == dest)          //If the source and destination are same we return 0
  {
    path[0] = s;
    len = 0;
    return 0;
  }
  bws[ind] = dest;
  source[sourceIndex] = 0;
  while (flag == 0)                            //runs the loop till source(since we are backtracing) is not found
  {
    mml = bws[bwsIndex];
    int y = 0;
    for (y = 0; y < calcsize(mapp(mml)); y++)   // used to access all the neighbours of mml
    {
      int x1 = 0, avai = 1;
      for (x1; x1 <= ind; x1++)
      {
        if (beer[mapp(mml)][y] == bws[x1])     //Checks if the accessed neighbour already exists in bws
        {

          avai = 0;
          break;
        }
      }
      if (avai == 1)                          //Adds the neighbour to bws if it does not already exist in it
      {
        ind++; sourceIndex++;
        bws[ind] = beer[mapp(mml)][y];
        source[sourceIndex] = bws[bwsIndex];
        if (bws[ind] == s)                    //If source (since we are backtracing) is achieved, we break from the loop
        {
          flag = 1;
          break;
        }
      }
    }
    if(bwsIndex==47)
    {
      return 2;
      }
    else
    bwsIndex++;
  }
  path[0] = bws[ind];
  path[1] = source[sourceIndex];
  int tien, pathIndex = 1;
  for (tien = sourceIndex - 1; tien >= 0; tien--)   //path is traced from the source to destination using bws and source
  { //and storing it in path
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
  len = pathIndex;                //len stores the size(that needs to be accessed) of the path array 
  return 1;
}*/
/*
   Function Name      : deterPath
   Input              : char start, int destNote
   Output             : None
   Logic              : It finds the most optimum path from our current position to the required MNP
   Example Call       : deterPath(a,29)
*/

/*int deterPath(char start, int destNote)
{
  char arr1[48];
  char arr2[48];
  giveMNP(destNote);
  int siz, index, destMNP, ind, temp, leastPath, small = 100;
  if(mnps==0)
  {
    return 0;
    }
  for (int goku = 0; goku < mnps; goku++)
  {
    destMNP = arr[goku];          //destMNP stores the MNP where the note is kept
    siz = calcSizeNe(destMNP);    //siz stores the no. of nodes that can approach the MNP
    ind = 0;
    //int temp, small, leastPath;
    int check;
    while (ind < siz)
    {
      check=findPath(start, neigh[destMNP - 1][ind]);
      if(check==2)
      {
        return 2;
        }
      for (index = 0; index <= len; index++)

        arr1[index] = path[index];

      if (small > index)   //Stores the smallest path out of all paths possible from one bot in arr2
      {
        small = index;
        number = destMNP;
        for (index = 0; index <= len; index++)
          arr2[index] = path[index];
      }
      ind++;
    }
  }
  for (temp = 0; temp <= small; temp++)
    path[temp] = arr2[temp];
  len = small;
return 1;
}
*/
int deterPath(char start, int destNote)
{
  char arr1[48];
  char arr2[48];
  giveMNP(destNote);
  int siz, index, destMNP, ind, temp, leastPath, small = 100;
  int pradyuman=1;//new addition its a flag of kuch to gadbad hai
  if(mnps==0)
  {
    return 0;
    }
  
  for (int goku = 0; goku < mnps; goku++)
  {
    destMNP = arr[goku];          /*destMNP stores the MNP where the note is kept*/
    siz = calcSizeNe(destMNP);    /*siz stores the no. of nodes that can approach the MNP*/
    ind = 0;
    //int temp, small, leastPath;
    int check;
    while (ind < siz)
    {
      check=findPath(start, neigh[destMNP - 1][ind]);
      if(check==2)
      {
        //pradyuman=1;
        }
      else
      {
        pradyuman=0;
        for (index = 0; index <= len; index++)
            arr1[index] = path[index];
        if (small > index)   /* Stores the smallest path out of all paths possible from one bot in arr2*/
        {
          small = index;
          number = destMNP;
          for (index = 0; index <= len; index++)
            arr2[index] = path[index];
        }
      }
      ind++;
    }
  }
  if(pradyuman==1)
  {
    return 2;
    }
  for(temp = 0; temp <= small; temp++)
    path[temp] = arr2[temp];
  len = small;
return 1;
}
/*
   Function Name      : insertNeighbour
   Input              : sa, re
   Output             : None
   Logic              : Reinserts the deleted positons when obstruction is by Firebird 
   Example Call       : insertNeighbour(a,b)
*/
void insertNeighbour(char sa, char re)
{
  int sizesa = calcsize(mapp(sa));   // Stores the no. of neighbours of sa
  beer[mapp(sa)][sizesa] = re;
  int sizere = calcsize(mapp(re));    //Repeat the procedure for re
  beer[mapp(re)][sizere] = sa;
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
  /*if (Serial.available() != 0)
  {
    meraSerial();
  }*/
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
  if (Serial.available() != 0)
  {
    meraSerial();
  }
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
  if (Serial.available() != 0)
  {
    meraSerial();
  }
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
  if (Serial.available() != 0)
  {
    meraSerial();
  }
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
  if (Serial.available() != 0)
  {
    meraSerial();
  }
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
  if ((ms + rs + ls) > 800)//wannachange 800
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
    /*if(Serial.available()!=0)
      {
      meraSerial();
      }*/
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
  //interrupts();
  if (Serial.available() != 0)
  {
    meraSerial();
  }
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
  //interrupts();
  if (Serial.available() != 0)
  {
    meraSerial();
  }
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
  {
    /*if(Serial.available()!=0)
      {
      meraSerial();
      }*/
    Line_Follow_Forw();
  }
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
  sarkar=1;
  int ReqdShaftCount = (Degrees - 10) / 4.090 ;                         //Rotate for degree-10
  ShaftCountRight = 0;
  while (ShaftCountRight < ReqdShaftCount)
    Left();
  ReqdShaftCount = (25) / 4.090 ;
  ShaftCountRight = 0;
  ms = analogRead(2);                                                   //Update sensor values
  while ((ms < Thresh) && (ShaftCountRight < ReqdShaftCount))           //Rotate while no black line detected
  {
    ms = analogRead(2);
    Left();
  }
  Stop();
  sarkar=2;
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
  sarkar=1;
  int ReqdShaftCount = (Degrees - 10) / 4.090 ;                         //Rotate for degree-10
  ShaftCountRight = 0;
  while (ShaftCountRight < ReqdShaftCount)
    Right();
  ReqdShaftCount = (28) / 4.090 ;
  ShaftCountRight = 0;
  ms = analogRead(2);                                                   //Update sensor values
  while ((ms < Thresh) && (ShaftCountRight < ReqdShaftCount))           //Rotate while no black line detected
  {
    ms = analogRead(2);
    Right();
  }
  Stop();
  sarkar=2;
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
    {
      /*
      if (Curr_Pos == 'c')
        Right_D(140);
      else
      */
        Right_D(120);
    }
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
   Function Name      : again_serial
   Input              : None
   Output             : None
   Logic              : Called if unreadable data is recieved by xbee. Again recieves the data which it was supposed to recieve through xbee by commanding Master to send data again, serially writing 1
   Example Call       : again_serial()
*/
void again_serial()
{

  counter++;
  while (!Serial.available())
  {}
  char its_pos = Serial.read();
  if (its_pos == Next_Pos)
  {
    delay(50);
    Serial.write(0);
    deleteNeighbour(Curr_Pos, Next_Pos);
    delay(50);
    Serial.write(125);
    //Serial2.println(125);//hataya serial print
    //Serial2.println("tu mere samne h");
    flagg = 0;
  }
  else if (its_pos <= 'x' && its_pos >= 'A')
  {
    delay(50);
    Serial.write(0);
    delay(50);//hatayi delay 100 to 50
    Serial.write(130);
    //Serial2.println(130);
    //Serial2.println("tu mere samne nhi h");
    deleteNeighbour(Next_Pos, Curr_Pos);
    delay(50);//hatayi delay from 100 to 50
    Serial.write(Curr_Pos);
    //Serial2.println(Curr_Pos);//hataya serial print
    //Serial2.println("le meri curr pos");
    delay(50);//hatayi delay from 100 to 50
    Serial.write(Next_Pos);
    //Serial2.println(Next_Pos);//hataya serial print
    //Serial2.println("le meri next pos");
    flagg = 1;
  }
  else
  {
    Serial.write(1);
    again_serial();
  }

}
/*
   Function Name      : hit
   Input              : None
   Output             : None
   Logic              : Checks the MNP to hit, Current Position, Next position and clockwise state of FB to find the case and then call striking left,right or middle function accordingly
   Example Call       : hit()
*/
void hit()
{
  //Buzzer(500);
  /*Serial2.println("hit function-next pos");//hataya serial print
  Serial2.println(Next_Pos);
  Serial2.println("curr pos");
  Serial2.println(Curr_Pos);
  Serial2.println("Prev pos");
  Serial2.println(Prev_Pos);
  Serial2.println("clockwise");
  Serial2.println(clockwise);
  Serial2.println("number");
  Serial2.println(number);*/
  Stop();
  //delay(100);//hatayi delay
  Back_Cm(3);
  myservo1.attach(11);
  myservo2.attach(12);
  if (Next_Pos >= 97 && Next_Pos <= 121 && number <= 24 && Curr_Pos >= 97) //Outer Circle
  {
    if (clockwise == 1) //Clockwise Direction
    {
      striking_left();
    }
    else if (clockwise == 0) //Anti-Clockwise Direction
    {
      striking_right();
    }
  }
  else if (Prev_Pos == Curr_Pos)
  {
    //Serial2.println("hi, I'm in same case");//hataya serial print
    //Serial2.println("number");
    //Serial2.println(number);
    //Serial2.println("number1");
    //Serial2.println(number1);
    if (number1 != number)
    {
      if (same == 1)
      {
        striking_right();
      }
      else if (same == 0)
      {
        striking_left();
      }
    }
    else if (number1 == number)
    {
      if (same == 1)
      {
        striking_left();
      }
      else if (same == 0)
      {
        striking_right();
      }
      else if (same == 2)
      {
      striking_middle();
      }
    }
    else if (same == 2)
    {
      striking_middle();
    }
  }
  else if ((Next_Pos == 99 || Next_Pos == 103 || Next_Pos == 107 || Next_Pos == 111 || Next_Pos == 115 || Next_Pos == 119) && Curr_Pos <= 89 && number <= 24)
  {
    striking_middle();
  }
  else if ((Next_Pos == 97 || Next_Pos == 101 || Next_Pos == 105 || Next_Pos == 109 || Next_Pos == 113 || Next_Pos == 117) && number >= 24 )
  {
    if (clockwise == 1) //Clockwise Direction
    {
      striking_right();
    }
    else if (clockwise == 0) //Anti-Clockwise Direction
    {
      striking_left();
    }
  }
  else if ((Next_Pos >= 65 && Next_Pos <= 89))
  {
    if (number == 25)
    {
      if ((Curr_Pos == 'w' && Next_Pos == 'A') || (Curr_Pos == 'A' && Next_Pos == 'C') || (Curr_Pos == 'C' && Next_Pos == 'B') || (Curr_Pos == 'U' && Next_Pos == 'C'))
      {
        striking_left();
      }
      else if ((Curr_Pos == 'c' && Next_Pos == 'B') || (Curr_Pos == 'B' && Next_Pos == 'C') || (Curr_Pos == 'C' && Next_Pos == 'A'))
      {
        striking_right();
      }
      else if ((Curr_Pos == 'G' && Next_Pos == 'B') || (Curr_Pos == 'D' && Next_Pos == 'C'))
      {
        striking_middle();
      }
    }

    else if (number == 27)
    {
      if ((Curr_Pos == 'c' && Next_Pos == 'G') || (Curr_Pos == 'G' && Next_Pos == 'F') || (Curr_Pos == 'F' && Next_Pos == 'H'))
      {
        striking_left();
      }
      else if ((Curr_Pos == 'g' && Next_Pos == 'H') || (Curr_Pos == 'H' && Next_Pos == 'F') || (Curr_Pos == 'F' && Next_Pos == 'G') || (Curr_Pos == 'O' && Next_Pos == 'F'))
      {
        striking_right();
      }
      else if ((Curr_Pos == 'B' && Next_Pos == 'G') || (Curr_Pos == 'E' && Next_Pos == 'F'))
      {
        striking_middle();
      }
    }

    else if (number == 28)
    {
      if ((Curr_Pos == 'g' && Next_Pos == 'I') || (Curr_Pos == 'I' && Next_Pos == 'O') || (Curr_Pos == 'O' && Next_Pos == 'J') || (Curr_Pos == 'F' && Next_Pos == 'O'))
      {
        striking_left();
      }
      else if ((Curr_Pos == 'k' && Next_Pos == 'J') || (Curr_Pos == 'J' && Next_Pos == 'O') || (Curr_Pos == 'O' && Next_Pos == 'I'))
      {
        striking_right();
      }
      else if ((Curr_Pos == 'N' && Next_Pos == 'O') || (Curr_Pos == 'K' && Next_Pos == 'J'))
      {
        striking_middle();
      }
    }

    else if (number == 33)
    {
      if ((Curr_Pos == 's' && Next_Pos == 'T') || (Curr_Pos == 'T' && Next_Pos == 'U') || (Curr_Pos == 'U' && Next_Pos == 'X'))
      {
        striking_left();
      }
      else if ((Curr_Pos == 'w' && Next_Pos == 'X') || (Curr_Pos == 'X' && Next_Pos == 'U') || (Curr_Pos == 'U' && Next_Pos == 'T') || (Curr_Pos == 'C' && Next_Pos == 'U'))
      {
        striking_right();
      }
      else if ((Curr_Pos == 'S' && Next_Pos == 'T') || (Curr_Pos == 'V' && Next_Pos == 'U'))
      {
        striking_middle();
      }
    }

    else if (number == 31)
    {
      if ((Curr_Pos == 'o' && Next_Pos == 'Q') || (Curr_Pos == 'Q' && Next_Pos == 'R') || (Curr_Pos == 'R' && Next_Pos == 'S') || (Curr_Pos == 'L' && Next_Pos == 'R'))
      {
        striking_left();
      }
      else if ((Curr_Pos == 's' && Next_Pos == 'S') || (Curr_Pos == 'S' && Next_Pos == 'R') || (Curr_Pos == 'R' && Next_Pos == 'Q'))
      {
        striking_right();
      }
      else if ((Curr_Pos == 'T' && Next_Pos == 'S') || (Curr_Pos == 'W' && Next_Pos == 'R'))
      {
        striking_middle();
      }
    }
    else if (number  == 30)
    {
      if ((Curr_Pos == 'k' && Next_Pos == 'K') || (Curr_Pos == 'K' && Next_Pos == 'L') || (Curr_Pos == 'L' && Next_Pos == 'P'))
      {
        striking_left();
      }
      else if ((Curr_Pos == 'o' && Next_Pos == 'P') || (Curr_Pos == 'P' && Next_Pos == 'L') || (Curr_Pos == 'L' && Next_Pos == 'K') || (Curr_Pos == 'R' && Next_Pos == 'L'))
      {
        striking_right();
      }
      else if ((Curr_Pos == 'J' && Next_Pos == 'K') || (Curr_Pos == 'M' && Next_Pos == 'L'))
      {
        striking_middle();
      }
    }

    else if (number  == 26)
    {
      if ((Curr_Pos == 'F' && Next_Pos == 'G') || (Curr_Pos == 'O' && Next_Pos == 'F') || (Curr_Pos == 'G' && Next_Pos == 'B') || (Curr_Pos == 'c' && Next_Pos == 'B') || (Curr_Pos == 'B' && Next_Pos == 'C') || (Curr_Pos == 'C' && Next_Pos == 'D'))
      {
        striking_left();
      }
      else if ((Curr_Pos == 'C' && Next_Pos == 'B') || (Curr_Pos == 'U' && Next_Pos == 'C') || (Curr_Pos == 'c' && Next_Pos == 'G') || (Curr_Pos == 'B' && Next_Pos == 'G') || (Curr_Pos == 'G' && Next_Pos == 'F') || (Curr_Pos == 'F' && Next_Pos == 'E'))
      {
        striking_right();
      }
      else if ((Curr_Pos == 'A' && Next_Pos == 'C') || (Curr_Pos == 'H' && Next_Pos == 'F'))
      {
        striking_middle();
      }
    }

    else if (number  == 29)
    {
      if ((Curr_Pos == 'K' && Next_Pos == 'J') || (Curr_Pos == 'J' && Next_Pos == 'O') || (Curr_Pos == 'k' && Next_Pos == 'J') || (Curr_Pos == 'O' && Next_Pos == 'N') || (Curr_Pos == 'L' && Next_Pos == 'K') || (Curr_Pos == 'R' && Next_Pos == 'L'))
      {
        striking_left();
      }
      else if ((Curr_Pos == 'N' && Next_Pos == 'O') || (Curr_Pos == 'J' && Next_Pos == 'K') || (Curr_Pos == 'O' && Next_Pos == 'J') || (Curr_Pos == 'K' && Next_Pos == 'L') || (Curr_Pos == 'F' && Next_Pos == 'O') || (Curr_Pos == 'k' && Next_Pos == 'K'))
      {
        striking_right();
      }
      else if ((Curr_Pos == 'P' && Next_Pos == 'L') || (Curr_Pos == 'I' && Next_Pos == 'O'))
      {
        striking_middle();
      }
    }

    else if (number  == 32)
    {
      if ((Curr_Pos == 'R' && Next_Pos == 'W') || (Curr_Pos == 'W' && Next_Pos == 'V') || (Curr_Pos == 'V' && Next_Pos == 'U') || (Curr_Pos == 'U' && Next_Pos == 'T') || (Curr_Pos == 'T' && Next_Pos == 'S') || (Curr_Pos == 'S' && Next_Pos == 'R') || (Curr_Pos == 's' && Next_Pos == 'S') || (Curr_Pos == 'C' && Next_Pos == 'U'))
      {
        striking_left();
      }
      else if ((Curr_Pos == 'W' && Next_Pos == 'R') || (Curr_Pos == 'V' && Next_Pos == 'W') || (Curr_Pos == 'U' && Next_Pos == 'V') || (Curr_Pos == 'T' && Next_Pos == 'U') || (Curr_Pos == 'S' && Next_Pos == 'T') || (Curr_Pos == 'R' && Next_Pos == 'S') || (Curr_Pos == 's' && Next_Pos == 'T') || (Curr_Pos == 'L' && Next_Pos == 'R'))
      {
        striking_right();
      }
      else if ((Curr_Pos == 'X' && Next_Pos == 'U') || (Curr_Pos == 'Q' && Next_Pos == 'R'))
      {
        striking_middle();
      }
    }
  }
  
  number1 = number;
  Forw_Cm(3);
  myservo1.detach();
  myservo2.detach();
}

 /*
   Function Name      : whoWillGoDash()
   Input              : None
   Output             : None
   Logic              : It finds and assigns the notes to master and slave initially according to ratio
   Example Call       : whoWillGoDash();
*/
void whoWillGoDash()
{
  char start1 = Curr_Pos;
  char start2 = its_pos;
  for(int ram=0;ram<sizeMNP-counter;ram++)
  {
    mnp[ram]=mnp[ram+counter];
  }
  sizeMNP=sizeMNP-counter;
  counter=0;
  
  int size1, size2, masOrSla, ratio;
  indM = 0;
  indS = 0;
  int check;
  char mayStart1, mayStart2;
  for (int kaneki = 0; kaneki < sizeMNP; kaneki++)
  {
    check=deterPath(start1, mnp[kaneki]);
    if(check==0)    //ek yahaan galti hai
    {
     for(int i=kaneki;i<(sizeMNP-1);i++)
     {
      mnp[i]=mnp[i+1];
      }     
      sizeMNP--;
      kaneki--;
    }
    else
    { 
    size1 = len;
    mayStart1 = path[len - 1];
    check = deterPath(start2, mnp[kaneki]);
    size2 = len;
    mayStart2 = path[len - 1];
    if (kaneki == 0)
    {
      if (size1 <= size2)
      {
        masterDrive[indM][1] = mnp[kaneki];
        masterDrive[indM][0] = kaneki;
        start1 = mayStart1;
        indM = indM + 1;
        masOrSla = 0;
        ratio = 2;
      }
      else
      {
        slaveDrive[indS][1] = mnp[kaneki];
        slaveDrive[indS][0] = kaneki;
        start2 = mayStart2;
        indS = indS + 1;
        masOrSla = 1;
        ratio = 2;
      }
    }
    else if (masOrSla == 0)  //Start from here
    {
      if (size1 <= size2 / ratio)
      {
        masterDrive[indM][1] = mnp[kaneki];
        masterDrive[indM][0] = kaneki;
        start1 = mayStart1;
        indM = indM + 1;
        ratio = ratio + 1;
      }
      else
      {
        slaveDrive[indS][1] = mnp[kaneki];
        slaveDrive[indS][0] = kaneki;
        start2 = mayStart2;
        indS = indS + 1;
        masOrSla = 1;
        ratio = 2;
      }
    }
    else
    {
      if (size2 <= size1 / ratio)
      {
        slaveDrive[indS][1] = mnp[kaneki];
        slaveDrive[indS][0] = kaneki;
        start2 = mayStart2;
        indS = indS + 1;
        ratio = ratio + 1;
      }
      else
      {
        masterDrive[indM][1] = mnp[kaneki];
        masterDrive[indM][0] = kaneki;
        start1 = mayStart1;
        indM = indM + 1;
        ratio = 2;
        masOrSla = 0;
      }
    }
  }
  //Serial2.println(indS);
  //Serial2.println(indM);
  }
  }

/*
   Function Name      : Traverse
   Input              : None
   Output             : None
   Logic              : Takes the bot to the desired location and updates its Current location
   Example Call       : Traverse()
*/

int Traverse()
{
  weed = 0;
  int byakugan;
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
      int flag = 0;
      obstacle = 0;  //Ab yahaan hai main kaam
      while(tenWala==10)//16tareek
      {
       Serial.write(100);
       while(!Serial.available())
       {}
       tenWala=Serial.read(); 
      }
      tenWala=10;
      while (!Serial.available())
      {}
      its_pos = Serial.read();
      //Serial2.println(its_pos);//hataya serial print
      //Serial2.println("ye uski position");
      if (its_pos == Next_Pos)
      {
        delay(50);
        Serial.write(0);
        deleteNeighbour(Curr_Pos, Next_Pos);
        delay(50);
        Serial.write(125);
        //Serial2.println(125);//hataya serial print
        //Serial2.println("tu mere samne h");
      }
      else if (its_pos <= 'x' && its_pos >= 'A')
      {
        delay(50);
        Serial.write(0);
        delay(100);
        Serial.write(130);
        //Serial2.println(130);hataya serial print
        //Serial2.println("tu mere samne nhi h");
        deleteNeighbour(Next_Pos, Curr_Pos);
        delay(50);//hatayi delay 100 to 50
        Serial.write(Curr_Pos);
        //Serial2.println(Curr_Pos);hataya serial print
        //Serial2.println("le meri curr pos");
        delay(50);//hatayi delay 100 to 50
        Serial.write(Next_Pos);
        //Serial2.println(Next_Pos);//hataya serial print
        //Serial2.println("le meri next pos");
        flag = 1;
      }
      else
      {
        Serial.write(1);
        again_serial();
        flag = flagg;
      }
      //Determine new path
      byakugan=deterPath(Curr_Pos, reach);
      //Serial2.println("byakugan ");//hataya serial print
      //Serial2.print(byakugan);
      if (flag == 0)
      {
        insertNeighbour(Next_Pos, Curr_Pos);
      }
      if(byakugan==2)  //ye kiya hai change
      {
      allign();
      return 2;
      }
      byakugan=Traverse();
      if(byakugan==2)  //ye kiya hai change
      {

      allign();
      return 2;
      }
      return 0;
    }
    allign();
  }
  return 1;
}

/*void Traverse()
{
  weed = 0;
  Prev_Pos = path[0];
  Curr_Pos = path[0];                                   //Update podition parameters
  Next_Pos = path[1];
  //ch = Next_Pos;
  delay(50);
  Serial2.println("ch");
  delay(50);
  Serial2.println(ch);
  delay(50);
  Serial2.println("reach");
  delay(50);
  Serial2.println(reach);
  delay(50);
  Serial2.println("path");
  delay(50);
  for(int gormint=0;gormint<len;gormint++)
  {
    Serial2.println(path[gormint]);
    delay(50);
  }
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
      int flag = 0;
      obstacle = 0;  //Ab yahaan hai main kaam
      Serial.write(100);
      while (!Serial.available())
      {}
      char its_pos = Serial.read();
      Serial2.println(its_pos);
      Serial2.println("ye uski position");
      if (its_pos == Next_Pos)
      {
        delay(50);
        Serial.write(0);
        deleteNeighbour(Curr_Pos, Next_Pos);
        delay(50);
        Serial.write(125);
        Serial2.println(125);
        Serial2.println("tu mere samne h");
      }
      else if (its_pos <= 'x' && its_pos >= 'A')
      {
        delay(50);
        Serial.write(0);
        delay(100);
        Serial.write(130);
        Serial2.println(130);
        Serial2.println("tu mere samne nhi h");
        deleteNeighbour(Next_Pos, Curr_Pos);
        delay(100);
        Serial.write(Curr_Pos);
        Serial2.println(Curr_Pos);
        Serial2.println("le meri curr pos");
        delay(100);
        Serial.write(Next_Pos);
        Serial2.println(Next_Pos);
        Serial2.println("le meri next pos");

         //while(!Serial.available())
          //{}
          //blast=Serial.read();
          //Serial2.println(blast);
          //Serial2.println("ye rha blast");
          //delay(100);
          //if(blast==1)
          //{
           //Stop();
           //again_serial();
          //}
        flag = 1;
      }
      else
      {
        Serial.write(1);
        again_serial();
        flag = flagg;
      }
      //Determine new path
      deterPath(Curr_Pos, reach);
      if (flag == 0)
      {
        insertNeighbour(Next_Pos, Curr_Pos);
      }
      Traverse();
      return 0;
    }
    allign();
  }
  return 1;
}*/


/*
   Function Name      : Play
   Input              : None
   Output             : None
   Logic              : Follows the path according to given mnps
   Example Call       : Play()
*/
int Play()
{
  moonfli=1;
  int check;
  
  for (pos; pos < slave_size; pos++)                  //Traverse the MNP array
  {
    reach = slaveDrive[pos][1];
    check=deterPath(ch, reach);    
    
    if(check==2)
    {
      while(tenWala==10)//16 tareek
      {
        Serial.write(145);
        while(!Serial.available())
        {}
        tenWala=Serial.read();  
      }
      tenWala=10;
      delay(50);//hatayi delay 500 to 50
      Serial.write(slaveDrive[pos][0]);
      delay(50);//hatayi delay 500 to 50
      Serial.write(reach);
      ch=Next_Pos;
    }
    else if(check==1)
    {
    check = Traverse();
    if(check==2)
    {
      while(tenWala==10)//16tareek
      {
       Serial.write(145);
       while(!Serial.available())
       {}
       tenWala=Serial.read(); 
      }
      tenWala=10;
      delay(50);//hatayi delay 500 to 50
      Serial.write(slaveDrive[pos][0]);
      delay(50);//hatayi delay 500 to 50
      Serial.write(reach);
      ch=Curr_Pos;
      continue;
      }
    while (counter != slaveDrive[pos][0])
    {
      if (Serial.available() != 0)
      {
        moonfli=2;
        meraSerial();
      }
    }
    //Buzzer(500);           //Short Buzzer
    delay(50);
    Buzzer(500);
    delay(50);
    if (counter == slave_size - 1)                 //Long Buzzer
      Buzzer(4500);
    counter = counter + 1;
    delay(500);
    while(tenWala==10)
    {
      Serial.write(200);
      while(!Serial.available())
      {}
      tenWala=Serial.read();
    }
    tenWala=10;
    delay(50);//hatayi delay 100 to 50
    ch = path[len - 1];                               //Update Current position
   }
  }
  return 0;
}
/*int Play()
{
  int check;
  for (pos; pos < slave_size; pos++)                  //Traverse the MNP array
  {
    reach = slaveDrive[pos][1];
    check=deterPath(ch, reach);
    if(check==2)
    {
      Serial.write(145);
      delay(500);
      Serial.write(slaveDrive[pos][0]);
      delay(500);
      Serial.write(slaveDrive[pos][1]);
      }
    else if(check==1)
    {  
    Traverse();
    while (counter != slaveDrive[pos][0])
    {
      if (Serial.available() != 0)
      {
        meraSerial();
      }
    }
    //Buzzer(500);           //Short Buzzer
    delay(50);
    Buzzer(500);
    delay(50);
    
    counter = counter + 1;
    delay(500);
    Serial.write(200);
    delay(100);
    ch = path[len - 1]; 
    }
  }
  return 0;
}*/
/*
   Function Name      : meraSerial
   Input              : None
   Output             : None
   Logic              : XBee serial communication. Cases 100-obstruction{130-obstacle & 125-Master}, 200- increment counter. if blast==1, send all data after obstruction detection again.
   Example Call       : meraSerial()
*/
void meraSerial()
{
  blast = 1;
  int recieve = Serial.read();
  Serial2.println("mera serial ka pehla");
  Serial2.println(recieve);
  chotaBuzzer(2500000);
  if (recieve == 100)
  {
    Stop();
    delay(50);
    Serial.write(9);
    delay(50);
    while (blast == 1)
    {
      Serial.write(Curr_Pos);//serially sends it's current position to the other bot
      Serial2.println(Curr_Pos);
      while (!Serial.available())
      {}
      blast = Serial.read();
      if (blast == 0)
      {
        while (!Serial.available())
        {}
        int rec = Serial.read();
        Serial2.println(rec);
        if (rec == 130)
        {
          while (!Serial.available())
          {}
          char sa = Serial.read();
          Serial2.println(sa);
          while (!Serial.available())
          {}
          char re = Serial.read();
          Serial2.println(re);
          deleteNeighbour(sa, re);
          delay(50);
          Rev();
          allign();
          whoWillGoDash();
        }
        else if (rec == 125)
        {
          delayMicroseconds(3000000);
        }
        break;
      }
    }
    

  }
  else if (recieve == 200)
  {
    delay(50);
    Serial.write(9);
    counter++;
  }
  else if(recieve==145)
  {
    Stop();
    delay(50);
    Serial.write(9);
    //Serial2.println("Got145");
    delay(50);
//    int bik=0;
//    while(bik==0)
//    {
//      Serial.write(sarkar);
//      delay(50);
//      while(Serial.available()==0)
//      {}
//      bik=Serial.read();
//    }
    while(!Serial.available())
    {}
    int posD= Serial.read();
    while(!Serial.available())
    {}
    int valD= Serial.read();
    delay(20);
    Serial2.println("Got145");
    //delay(50);//hataya serial print
    //Serial2.println("posD");
    delay(20);
    Serial2.println(posD);
    //delay(50);//hataya serial print
    //Serial2.println("valD");
    delay(20);
    Serial2.println(valD);
    //delay(50);//hataya serial print
    //Serial2.println("ras=pos");
    delay(20);
    Serial2.println(pos);
    //delay(50);//hataya serial print
    //Serial2.println("slave_size");
    delay(20);
    Serial2.println(slave_size);
    delay(20);
    for(int ras=pos;ras<slave_size;ras++)
    {
      if(slaveDrive[ras][0]>posD)   //ye change kiya divyansh important
      {
        //delay(50);//hataya serial print
        //Serial2.println("I'm in if");
        //delay(50);
        //Serial2.println("slave_size");
        delay(20);
        Serial2.println(slave_size);
        //delay(50);//hataya serial print
        //Serial2.println("ras");
        delay(20);
        Serial2.println(ras);
        //delay(50);//hatayi delay
        for(int temp=(slave_size-1);temp>=ras;temp--)
        {
          //Serial2.println("i'm in for");
          //delay(50);//hataya serial print
          slaveDrive[temp+1][0]=slaveDrive[temp][0];
          slaveDrive[temp+1][1]=slaveDrive[temp][1];
          }
         slave_size++;
         slaveDrive[ras][0]=posD;
         slaveDrive[ras][1]=valD;
         reach = slaveDrive[pos][1];
         Rev();
         if(moonfli==2)
         ch=Next_Pos;  //dekh idhar
         else
         ch= Curr_Pos;
         //delay(50);//hataya serial print
         //Serial2.println("sras0");
         //delay(50);
         //Serial2.println(slaveDrive[ras][0]);
         //delay(50);
         //Serial2.println("sras1");
         //delay(50);
         //Serial2.println(slaveDrive[ras][1]);
         //delay(50);
         //Serial2.println("reach");
         //delay(50);
         //Serial2.println(reach);
         //delay(50);
         //Serial2.println("ch");
         //delay(50);
         //Serial2.println(ch);
         //delay(50);
         //Serial2.println("Slave drive");
         //delay(50);
         //for(int inazuma=0;inazuma<slave_size;inazuma++)
         //{
          //Serial2.print(slaveDrive[inazuma][0]);
          //delay(50);
          //Serial2.print(" ");
          //delay(50);
          //Serial2.println(slaveDrive[inazuma][1]);
          //delay(50);
         //}
         allign();
         Play(); 
        } 
      }
    }
  else
  {
    delay(50);
    Serial.write(10);  //16tareek
    delay(50);
    Serial.println("in case 10");
  }
}

/*
   Function Name      : getSlave
   Input              : None
   Output             : None
   Logic              : Gets serial data from python via USB
   Example Call       : getSlave()
*/
void getSlave()
{
  while (!Serial.available())
  {}
  int size = Serial.read();
  indS = size;
  //Serial2.println(size);//hataya serial print
  for (int rasengan = 0; rasengan < size; rasengan++)
  {
    while (!Serial.available())
    {}
    slaveDrive[rasengan][0] = Serial.read();
    while (!Serial.available())
    {}
    slaveDrive[rasengan][1] = Serial.read();
    //Serial2.println(slaveDrive[rasengan][0]);//hataya serial print
    //Serial2.println(slaveDrive[rasengan][1]);
  }
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

  //serial comm starts
  UCSR0B = 0x00;
  UCSR0A = 0x00;
  UCSR0C = 0x06;
  UBRR0L = 0x5F;
  UBRR0H = 0x00;
  UCSR0B = 0x98;

  UCSR2B = 0x00;
  UCSR2A = 0x00;
  UCSR2C = 0x06;
  UBRR2L = 0x5F;
  UBRR2H = 0x00;
  UCSR2B = 0x98;
  //serial comm initaization ends
  pinMode(A6, INPUT);                                     //A6 is the proximity sensor in front of the bot
  pinMode(3, INPUT);                                      //1,2,3 are the White line sensors
  pinMode(2, INPUT);
  pinMode(1, INPUT);
  pinMode(34, OUTPUT);                                    //Buzzer Pin
  pinMode(45, OUTPUT);                                    //PWM inputs of motor drivers
  pinMode(46, OUTPUT);
  pinMode(18, OUTPUT);
  pinMode(19, INPUT);
  analogWrite(45, 255); //wanna change 225
  // Interrupts Initialization
  attachInterrupt(digitalPinToInterrupt(2), Count_Left, FALLING);  //other interrupts 18,19,20,21
  attachInterrupt(digitalPinToInterrupt(3), Count_Right, FALLING);

  DDRA = 0x0F;
  DDRL = DDRL | 0x18;
  PORTL = PORTL | 0x18;
  DDRE  = DDRE & 0xEF;
  PORTE = PORTE | 0x10;
  DDRE  = DDRE & 0xDF;
  PORTE = PORTE | 0x20;
  myservo1.attach(11);
  myservo2.attach(12);
  myservo1.write(0);
  delay(200);
  myservo2.write(180);
  delay(200);
  //striking_right();
  myservo1.detach();
  myservo2.detach();
  delay(100);//change
  pinMode(14,OUTPUT);
  int shu;
  getSlave();
  slave_size=indS;
  Play();                                                             //Let the Game begin :)
  //delay(500);//hata buzzer //hatayi delay
  //Buzzer(1000);
}
/*
   Function Name      : loop
   Input              : None
   Output             : None
   Logic              : None
   Example Call       : loop()
*/
void loop() {
  // This runs after the completion of all notestriking by one of the robot. In case the other finds out that it's paths are all blocked, it have to pass that info the stopped bot, covered in this case
  if(Serial.available()!=0)
  {
    meraSerial();
    Stop();
  }
}




