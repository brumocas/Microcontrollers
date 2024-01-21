#include <Servo.h>

#define TIME_STEP 5
#define TIME_MAX ((2500 - 500)/TIME_STEP) + 2

struct Servo1
{   
  Servo servo;
  // To access by angle use hash table -> 1000ms equals 0 degrees
  int time[TIME_MAX] =  {
    500, 505, 510, 515, 520, 525, 530, 535, 540, 545,
    550, 555, 560, 565, 570, 575, 580, 585, 590, 595,
    600, 605, 610, 615, 620, 625, 630, 635, 640, 645,
    650, 655, 660, 665, 670, 675, 680, 685, 690, 695,
    700, 705, 710, 715, 720, 725, 730, 735, 740, 745,
    750, 755, 760, 765, 770, 775, 780, 785, 790, 795,
    800, 805, 810, 815, 820, 825, 830, 835, 840, 845,
    850, 855, 860, 865, 870, 875, 880, 885, 890, 895,
    900, 905, 910, 915, 920, 925, 930, 935, 940, 945,
    950, 955, 960, 965, 970, 975, 980, 985, 990, 995,
    1000, 1005, 1010, 1015, 1020, 1025, 1030, 1035, 1040, 1045,
    1050, 1055, 1060, 1065, 1070, 1075, 1080, 1085, 1090, 1095,
    1100, 1105, 1110, 1115, 1120, 1125, 1130, 1135, 1140, 1145,
    1150, 1155, 1160, 1165, 1170, 1175, 1180, 1185, 1190, 1195,
    1200, 1205, 1210, 1215, 1220, 1225, 1230, 1235, 1240, 1245,
    1250, 1255, 1260, 1265, 1270, 1275, 1280, 1285, 1290, 1295,
    1300, 1305, 1310, 1315, 1320, 1325, 1330, 1335, 1340, 1345,
    1350, 1355, 1360, 1365, 1370, 1375, 1380, 1385, 1390, 1395,
    1400, 1405, 1410, 1415, 1420, 1425, 1430, 1435, 1440, 1445,
    1450, 1455, 1460, 1465, 1470, 1475, 1480, 1485, 1490, 1495,
    1500, 1505, 1510, 1515, 1520, 1525, 1530, 1535, 1540, 1545,
    1550, 1555, 1560, 1565, 1570, 1575, 1580, 1585, 1590, 1595,
    1600, 1605, 1610, 1615, 1620, 1625, 1630, 1635, 1640, 1645,
    1650, 1655, 1660, 1665, 1670, 1675, 1680, 1685, 1690, 1695,
    1700, 1705, 1710, 1715, 1720, 1725, 1730, 1735, 1740, 1745,
    1750, 1755, 1760, 1765, 1770, 1775, 1780, 1785, 1790, 1795,
    1800, 1805, 1810, 1815, 1820, 1825, 1830, 1835, 1840, 1845,
    1850, 1855, 1860, 1865, 1870, 1875, 1880, 1885, 1890, 1895,
    1900, 1905, 1910, 1915, 1920, 1925, 1930, 1935, 1940, 1945,
    1950, 1955, 1960, 1965, 1970, 1975, 1980, 1985, 1990, 1995,
    2000, 2005, 2010, 2015, 2020, 2025, 2030, 2035, 2040, 2045,
    2050, 2055, 2060, 2065, 2070, 2075, 2080, 2085, 2090, 2095,
    2100, 2105, 2110, 2115, 2120, 2125, 2130, 2135, 2140, 2145,
    2150, 2155, 2160, 2165, 2170, 2175, 2180, 2185, 2190, 2195,
    2200, 2205, 2210, 2215, 2220, 2225, 2230, 2235, 2240, 2245,
    2250, 2255, 2260, 2265, 2270, 2275, 2280, 2285, 2290, 2295,
    2300, 2305, 2310, 2315, 2320, 2325, 2330, 2335, 2340, 2345,
    2350, 2355, 2360, 2365, 2370, 2375, 2380, 2385, 2390, 2395,
    2400, 2405, 2410, 2415, 2420, 2425, 2430, 2435, 2440, 2445,
    2450, 2455, 2460, 2465, 2470, 2475, 2480, 2485, 2490, 2495,
    2500, 2505
};

  // Hash function to get angle corresponding PWM
  int getPWM(int angle){
  
  if(angle >= 130){
    angle = 130;
  } else if (angle <= 0){
    angle = 0;
  }
  int hash = map(angle, 0, 180, 0, TIME_MAX - 1);
  return time[hash];
  }

}; 
Servo1 s1;





struct Servo2
{   
  Servo servo;
  // To access by angle use hash table -> 1000ms equals 0 degrees
  int time[TIME_MAX] =  {
    500, 505, 510, 515, 520, 525, 530, 535, 540, 545,
    550, 555, 560, 565, 570, 575, 580, 585, 590, 595,
    600, 605, 610, 615, 620, 625, 630, 635, 640, 645,
    650, 655, 660, 665, 670, 675, 680, 685, 690, 695,
    700, 705, 710, 715, 720, 725, 730, 735, 740, 745,
    750, 755, 760, 765, 770, 775, 780, 785, 790, 795,
    800, 805, 810, 815, 820, 825, 830, 835, 840, 845,
    850, 855, 860, 865, 870, 875, 880, 885, 890, 895,
    900, 905, 910, 915, 920, 925, 930, 935, 940, 945,
    950, 955, 960, 965, 970, 975, 980, 985, 990, 995,
    1000, 1005, 1010, 1015, 1020, 1025, 1030, 1035, 1040, 1045,
    1050, 1055, 1060, 1065, 1070, 1075, 1080, 1085, 1090, 1095,
    1100, 1105, 1110, 1115, 1120, 1125, 1130, 1135, 1140, 1145,
    1150, 1155, 1160, 1165, 1170, 1175, 1180, 1185, 1190, 1195,
    1200, 1205, 1210, 1215, 1220, 1225, 1230, 1235, 1240, 1245,
    1250, 1255, 1260, 1265, 1270, 1275, 1280, 1285, 1290, 1295,
    1300, 1305, 1310, 1315, 1320, 1325, 1330, 1335, 1340, 1345,
    1350, 1355, 1360, 1365, 1370, 1375, 1380, 1385, 1390, 1395,
    1400, 1405, 1410, 1415, 1420, 1425, 1430, 1435, 1440, 1445,
    1450, 1455, 1460, 1465, 1470, 1475, 1480, 1485, 1490, 1495,
    1500, 1505, 1510, 1515, 1520, 1525, 1530, 1535, 1540, 1545,
    1550, 1555, 1560, 1565, 1570, 1575, 1580, 1585, 1590, 1595,
    1600, 1605, 1610, 1615, 1620, 1625, 1630, 1635, 1640, 1645,
    1650, 1655, 1660, 1665, 1670, 1675, 1680, 1685, 1690, 1695,
    1700, 1705, 1710, 1715, 1720, 1725, 1730, 1735, 1740, 1745,
    1750, 1755, 1760, 1765, 1770, 1775, 1780, 1785, 1790, 1795,
    1800, 1805, 1810, 1815, 1820, 1825, 1830, 1835, 1840, 1845,
    1850, 1855, 1860, 1865, 1870, 1875, 1880, 1885, 1890, 1895,
    1900, 1905, 1910, 1915, 1920, 1925, 1930, 1935, 1940, 1945,
    1950, 1955, 1960, 1965, 1970, 1975, 1980, 1985, 1990, 1995,
    2000, 2005, 2010, 2015, 2020, 2025, 2030, 2035, 2040, 2045,
    2050, 2055, 2060, 2065, 2070, 2075, 2080, 2085, 2090, 2095,
    2100, 2105, 2110, 2115, 2120, 2125, 2130, 2135, 2140, 2145,
    2150, 2155, 2160, 2165, 2170, 2175, 2180, 2185, 2190, 2195,
    2200, 2205, 2210, 2215, 2220, 2225, 2230, 2235, 2240, 2245,
    2250, 2255, 2260, 2265, 2270, 2275, 2280, 2285, 2290, 2295,
    2300, 2305, 2310, 2315, 2320, 2325, 2330, 2335, 2340, 2345,
    2350, 2355, 2360, 2365, 2370, 2375, 2380, 2385, 2390, 2395,
    2400, 2405, 2410, 2415, 2420, 2425, 2430, 2435, 2440, 2445,
    2450, 2455, 2460, 2465, 2470, 2475, 2480, 2485, 2490, 2495,
    2500, 2505
};

  // Hash function to get angle corresponding PWM
  int getPWM(int angle){
  
  /* 
  if(angle >= 100){
    angle = 100;
  } else if (angle <= 0){
    angle = 0;
  }
  */
  int hash = map(angle, 0, 180, 0, TIME_MAX - 1);
  Serial.print(" ");
  Serial.print(time[hash]);
  return time[hash];
  }

}; 
Servo2 s2;

struct Servo3
{   
  Servo servo;
  float angle[TIME_MAX];
  // To access by angle use hash table -> 1000ms equals 0 degrees
  int time[TIME_MAX] =  {
    500, 505, 510, 515, 520, 525, 530, 535, 540, 545,
    550, 555, 560, 565, 570, 575, 580, 585, 590, 595,
    600, 605, 610, 615, 620, 625, 630, 635, 640, 645,
    650, 655, 660, 665, 670, 675, 680, 685, 690, 695,
    700, 705, 710, 715, 720, 725, 730, 735, 740, 745,
    750, 755, 760, 765, 770, 775, 780, 785, 790, 795,
    800, 805, 810, 815, 820, 825, 830, 835, 840, 845,
    850, 855, 860, 865, 870, 875, 880, 885, 890, 895,
    900, 905, 910, 915, 920, 925, 930, 935, 940, 945,
    950, 955, 960, 965, 970, 975, 980, 985, 990, 995,
    1000, 1005, 1010, 1015, 1020, 1025, 1030, 1035, 1040, 1045,
    1050, 1055, 1060, 1065, 1070, 1075, 1080, 1085, 1090, 1095,
    1100, 1105, 1110, 1115, 1120, 1125, 1130, 1135, 1140, 1145,
    1150, 1155, 1160, 1165, 1170, 1175, 1180, 1185, 1190, 1195,
    1200, 1205, 1210, 1215, 1220, 1225, 1230, 1235, 1240, 1245,
    1250, 1255, 1260, 1265, 1270, 1275, 1280, 1285, 1290, 1295,
    1300, 1305, 1310, 1315, 1320, 1325, 1330, 1335, 1340, 1345,
    1350, 1355, 1360, 1365, 1370, 1375, 1380, 1385, 1390, 1395,
    1400, 1405, 1410, 1415, 1420, 1425, 1430, 1435, 1440, 1445,
    1450, 1455, 1460, 1465, 1470, 1475, 1480, 1485, 1490, 1495,
    1500, 1505, 1510, 1515, 1520, 1525, 1530, 1535, 1540, 1545,
    1550, 1555, 1560, 1565, 1570, 1575, 1580, 1585, 1590, 1595,
    1600, 1605, 1610, 1615, 1620, 1625, 1630, 1635, 1640, 1645,
    1650, 1655, 1660, 1665, 1670, 1675, 1680, 1685, 1690, 1695,
    1700, 1705, 1710, 1715, 1720, 1725, 1730, 1735, 1740, 1745,
    1750, 1755, 1760, 1765, 1770, 1775, 1780, 1785, 1790, 1795,
    1800, 1805, 1810, 1815, 1820, 1825, 1830, 1835, 1840, 1845,
    1850, 1855, 1860, 1865, 1870, 1875, 1880, 1885, 1890, 1895,
    1900, 1905, 1910, 1915, 1920, 1925, 1930, 1935, 1940, 1945,
    1950, 1955, 1960, 1965, 1970, 1975, 1980, 1985, 1990, 1995,
    2000, 2005, 2010, 2015, 2020, 2025, 2030, 2035, 2040, 2045,
    2050, 2055, 2060, 2065, 2070, 2075, 2080, 2085, 2090, 2095,
    2100, 2105, 2110, 2115, 2120, 2125, 2130, 2135, 2140, 2145,
    2150, 2155, 2160, 2165, 2170, 2175, 2180, 2185, 2190, 2195,
    2200, 2205, 2210, 2215, 2220, 2225, 2230, 2235, 2240, 2245,
    2250, 2255, 2260, 2265, 2270, 2275, 2280, 2285, 2290, 2295,
    2300, 2305, 2310, 2315, 2320, 2325, 2330, 2335, 2340, 2345,
    2350, 2355, 2360, 2365, 2370, 2375, 2380, 2385, 2390, 2395,
    2400, 2405, 2410, 2415, 2420, 2425, 2430, 2435, 2440, 2445,
    2450, 2455, 2460, 2465, 2470, 2475, 2480, 2485, 2490, 2495,
    2500, 2505
    };

  // Hash function to get angle corresponding PWM
  int getPWM(int angle){
  int hash = map(angle, 0, 180, 0, TIME_MAX - 1);
  return time[hash];
  }

}; 
Servo3 s3;

struct Servo4
{   
  Servo servo;
  float angle[TIME_MAX];
  // To access by angle use hash table -> 1000ms equals 0 degrees
  int time[TIME_MAX] =  {
    500, 505, 510, 515, 520, 525, 530, 535, 540, 545,
    550, 555, 560, 565, 570, 575, 580, 585, 590, 595,
    600, 605, 610, 615, 620, 625, 630, 635, 640, 645,
    650, 655, 660, 665, 670, 675, 680, 685, 690, 695,
    700, 705, 710, 715, 720, 725, 730, 735, 740, 745,
    750, 755, 760, 765, 770, 775, 780, 785, 790, 795,
    800, 805, 810, 815, 820, 825, 830, 835, 840, 845,
    850, 855, 860, 865, 870, 875, 880, 885, 890, 895,
    900, 905, 910, 915, 920, 925, 930, 935, 940, 945,
    950, 955, 960, 965, 970, 975, 980, 985, 990, 995,
    1000, 1005, 1010, 1015, 1020, 1025, 1030, 1035, 1040, 1045,
    1050, 1055, 1060, 1065, 1070, 1075, 1080, 1085, 1090, 1095,
    1100, 1105, 1110, 1115, 1120, 1125, 1130, 1135, 1140, 1145,
    1150, 1155, 1160, 1165, 1170, 1175, 1180, 1185, 1190, 1195,
    1200, 1205, 1210, 1215, 1220, 1225, 1230, 1235, 1240, 1245,
    1250, 1255, 1260, 1265, 1270, 1275, 1280, 1285, 1290, 1295,
    1300, 1305, 1310, 1315, 1320, 1325, 1330, 1335, 1340, 1345,
    1350, 1355, 1360, 1365, 1370, 1375, 1380, 1385, 1390, 1395,
    1400, 1405, 1410, 1415, 1420, 1425, 1430, 1435, 1440, 1445,
    1450, 1455, 1460, 1465, 1470, 1475, 1480, 1485, 1490, 1495,
    1500, 1505, 1510, 1515, 1520, 1525, 1530, 1535, 1540, 1545,
    1550, 1555, 1560, 1565, 1570, 1575, 1580, 1585, 1590, 1595,
    1600, 1605, 1610, 1615, 1620, 1625, 1630, 1635, 1640, 1645,
    1650, 1655, 1660, 1665, 1670, 1675, 1680, 1685, 1690, 1695,
    1700, 1705, 1710, 1715, 1720, 1725, 1730, 1735, 1740, 1745,
    1750, 1755, 1760, 1765, 1770, 1775, 1780, 1785, 1790, 1795,
    1800, 1805, 1810, 1815, 1820, 1825, 1830, 1835, 1840, 1845,
    1850, 1855, 1860, 1865, 1870, 1875, 1880, 1885, 1890, 1895,
    1900, 1905, 1910, 1915, 1920, 1925, 1930, 1935, 1940, 1945,
    1950, 1955, 1960, 1965, 1970, 1975, 1980, 1985, 1990, 1995,
    2000, 2005, 2010, 2015, 2020, 2025, 2030, 2035, 2040, 2045,
    2050, 2055, 2060, 2065, 2070, 2075, 2080, 2085, 2090, 2095,
    2100, 2105, 2110, 2115, 2120, 2125, 2130, 2135, 2140, 2145,
    2150, 2155, 2160, 2165, 2170, 2175, 2180, 2185, 2190, 2195,
    2200, 2205, 2210, 2215, 2220, 2225, 2230, 2235, 2240, 2245,
    2250, 2255, 2260, 2265, 2270, 2275, 2280, 2285, 2290, 2295,
    2300, 2305, 2310, 2315, 2320, 2325, 2330, 2335, 2340, 2345,
    2350, 2355, 2360, 2365, 2370, 2375, 2380, 2385, 2390, 2395,
    2400, 2405, 2410, 2415, 2420, 2425, 2430, 2435, 2440, 2445,
    2450, 2455, 2460, 2465, 2470, 2475, 2480, 2485, 2490, 2495,
    2500, 2505
    };

  // Hash function to get angle corresponding PWM
  int getPWM(int angle){
    int hash = map(angle, 0, 180, 0, TIME_MAX - 1);
    return time[hash];
  }

}; 
Servo4 s4;