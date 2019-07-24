#include "include.h"

extern uint8 img_handle[CAMERA_H][CAMERA_W];  
extern uint8 KeyValue;

void ScreenShow(void)
{
//   uint8 i;
//   uint8* string;
//  uint16 H;
//  uint16 W;
  LCD_img();
//  for(i=0;i<119;i++)
//  {
//      W=(uint16)(79);
//      H=i;
//      LCD_Fill(W,H,W,H,RED);
//  }
//   CenterShow();//œ‘ æ÷–œﬂ
//   LeftShow();  //œ‘ æ◊Û±ﬂ‘µ£®∫Ï£©
//   RightShow(); //œ‘ æ”“±ﬂ‘µ£®ª∆£©
//   LCD_ShowNum(0,110,(int32)(RED1),2);
//   LCD_ShowNum(20,110,(int32)(RED2),2);
//   LCD_ShowNum(40,110,(int32)(RED3),2);
//   LCD_ShowNum(60,110,(int32)(RED4),2);
//   
//   LCD_ShowNum(80,110,(int32)(Blackline_flag),2);
//   LCD_ShowNum(100,110,(int32)(Car_Stop_flag),2);
// LCD_ShowNum(0,110,(int32)(Track_complexity),4);
//   LCD_ShowNum(0,110,(int32)(Prospect_See),4);
//  switch(cross_flag)
//  {
//  case NUCROSS:
//    string = "N";break;
//  case CRO_IN:
//    string = "I";break;
//  case CRO_BEFORE:
//    string = "B";
//  }
//  LCD_ShowString(0,112,string);
//  switch(circle_flag)
//  {
//  case LEFT:
//    string =  "Lf";break;
//  case LCIRCLE:
//    string =  "LC";break;
//  case LCIRCLE_IN:
//    string = "Li";break;
//  case LCIRCLE_OUT:
//    string = "Lb";break;
//  case RIGHT:
//    string = "Rf";break;
//  case RCIRCLE:
//    string = "RC";break;
//  case RCIRCLE_IN:
//    string = "Ri";break;
//  case RCIRCLE_OUT:
//    string = "Rb";break;
//  default:
//    string = "NU";
//  }
//  LCD_ShowString(36,112,string);
//  
//  LCD_ShowNum(54,112,(int32)(srcnuma),3);
//  LCD_ShowNum(81,112,(int32)(srcnumb),3);
//  
//  if(camer.error < 0)
//  {
//    LCD_ShowChar(108,112,'-',0);
//    LCD_ShowNum(117,112,(int32)(-1*camer.error),3);
//  }
//  else
//    LCD_ShowNum(117,112,(int32)(camer.error),3);
//    LCD_ShowNum(40,110,(int32)(camer.error),4);
//   LCD_ShowNum(60,110,(int32)(Mline[119]),3);
  // LCD_ShowNum(120,110,(int32)(motor.speed_set),4);
}

void LCD_img(void)
{
//  uint8* string;
  int i,j;
//  int16 image_threshold;
  ////////?????????/////////////
  uint8 (*img_point)[160]=img_handle;
  uint8* imgh_p;
  int16* m=Mline;
  int16* l=Lline;
  int16* r=Rline;
  for(i=0;i<CAMERA_H;i++)
  {
    imgh_p = *(img_point);
    if(*m > -1 && *m < 160)
      *(imgh_p+(int)(*m)) = 20;
    if(*l > -1 && *l < 160)
      *(imgh_p+*l) = 10;
    if(*r > -1 && *r < 160)
      *(imgh_p+*r) = 30;
    img_point ++;
    m ++;
    l ++;
    r ++;
  }
  ////////////????/////////////////////
  Address_set(0,0,CAMERA_W-1,CAMERA_H-1);
  for(i=0;i<CAMERA_H;i++)
  {
    for(j=0;j<CAMERA_W;j++)
    {
////       if(i == srcnuma && j>=border[i][2] && j<=border[i][3])  LCD_Write_Data2(RED);
      if(j == 79)       LCD_Write_Data2(RED);
      else
      {
        switch(img_handle[i][j])
        {
        case 0:
          LCD_Write_Data2(BLACK);
          break;
        case 10:
          LCD_Write_Data2(RED);
          break;
        case 20:
          LCD_Write_Data2(BLUE);
          break;
        case 30:
          LCD_Write_Data2(RED);
          break;
        case 255:
          LCD_Write_Data2(WHITE);
          break;
        default:
          LCD_Write_Data2(BLACK);
        }
      }
    }
  }
}
//void LCD_img(void)
//{
//  int i,j;
//  Address_set(0,0,OV7725_EAGLE_W-1,OV7725_EAGLE_H-1);
//  for(i=0;i<CAMERA_H;i++)
//  {
//    for(j=0;j<CAMERA_W;j++)
//    {
//      if(img_handle[i][j]==255)
//      {
//        LCD_Write_Data2(WHITE);
//      }
//      if(img_handle[i][j]==0)
//      {
//        LCD_Write_Data2(BLACK);
//      }
//    }
//  }
//  
////  Address_set(0,20,OV7725_EAGLE_W-1,105);
////  for(i=35;i<120;i++)
////  {
////    for(j=0;j<CAMERA_W;j++)
////    {
////      if(img_handle[i][j]==255)
////      {
////        LCD_Write_Data2(WHITE);
////      }
////      if(img_handle[i][j]==0)
////      {
////        LCD_Write_Data2(BLACK);
////      }
////    }
////  }
//}

/***************÷–œﬂœ‘ æ*********************/
void CenterShow(void)
{
  uint8 i;
  uint16 H;
  uint16 W;
  for(i=0;i<119;i++)
  {
    if((Mline[i]>=0)&&(Mline[i]<=159))
    {
      W=(uint16)(Mline[i]);
      H=i;
      LCD_Fill(W,H,W,H,BLUE);
    }
      W=(uint16)(79);
      H=i;
      LCD_Fill(W,H,W,H,RED);
  }
  for(i=0;i<159;i++)
  {
      W=i;
      H=(uint16)(120-90);
      LCD_Fill(W,H,W,H,RED);
  }
  for(i=0;i<159;i++)
  {
      W=i;
      H=(uint16)(120-80);
      LCD_Fill(W,H,W,H,RED);
  }
}
/****************◊Û±ﬂ‘µœ‘ æ*********************/
void LeftShow(void)
{
  uint8 i;
  uint16 H;
  uint16 W;
  for(i=0;i<119;i++)
  {
    if(((Lline[i]+1)>=0)&&((Lline[i]+1)<=159))
    {
      W=(uint16)(Lline[i]+1);
      H=i;
      LCD_Fill(W,H,W,H,RED);
    }
//    if((Lline[i+35]>=0)&&(Lline[i+35]<=159))
//    {
//      W=(uint16)(Lline[i+35]);
//      H=20+i;
//      LCD_Fill(W,H,W,H,RED);
//    }
//    else if(Lline[i+35]<0)
//    {
//     W=0;
//     H=20+i;
//     LCD_Fill(W,H,W,H,RED);
//    }
//    else if(Lline[i+35]>159)
//    {
//     W=159;
//     H=20+i;
//     LCD_Fill(W,H,W,H,RED);
//    }
  }
}



/****************”“±ﬂ‘µœ‘ æ*********************/
void RightShow(void)
{
  uint8 i;
  uint16 H;
  uint16 W;
  for(i=0;i<119;i++)
  {
    if(((Rline[i]-1)>=0)&&((Rline[i]-1)<=159))
    {
      W=(uint16)(Rline[i]-1);
      H=i;
      LCD_Fill(W,H,W,H,RED);
    }
//    if((Rline[i+35]>=0)&&(Rline[i+35]<=159))
//    {
//      W=(uint16)(Rline[i+35]);
//      H=20+i;
//      LCD_Fill(W,H,W,H,YELLOW);
//    }
//    else if(Rline[i+35]<0)
//    {
//     W=0;
//     H=20+i;
//     LCD_Fill(W,H,W,H,YELLOW);
//    }
//    else if(Rline[i+35]>159)
//    {
//     W=159;
//     H=20+i;
//     LCD_Fill(W,H,W,H,YELLOW);
//    }
  }
}