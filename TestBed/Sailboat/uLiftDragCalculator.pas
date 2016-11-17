unit uLiftDragCalculator;   // Lift drag in slender, flat plate
 //------------------------//
// Om: For aspect rate = 0.2 .. Used a paddle table, just to get the aero dynamic  feeling of CD, CL
// History:
// by Omar Out16: start

interface

 //---------------------------------------------------------------//\
// returns aerodynamic coefficients (lift,drag) for a flat plate ()  \
//                                                                    \
//                       ^  Lift                                       \
//          ***          |                                              \
//             ***       |                                               \
//      Alpha //   ***   |      Drag                                      \
//      ------------- ****=====> --------------> wind direction           /
//                         ***                                           /
//                             ***                                      /
//                                ***                                  /
//                                                                    /
 //---------------------------------------------------------------// /
                                                                   //

//  Lift/ Drag forces in 2D can be calculated by:
//    FL := 1/2 * Ro * W^2 * L * CL
//    FD := 1/2 * Ro * W^2 * L * CD
//  where: FD=drag force FL=lift force Ro=fluid density (air or water)
//         W=wind speed  L=airfoil chord (or area for 3d)
//         CL=Lift coeff CD=Drag coeff
//  coefficients are found by airfoil tests (table based)
//  Lift grows fast with angle, but at some point the airfoil stols and lift falls dramatically.
//  Drag on the other hand grows more or less steadily, and is small for low angles of attack
//  This is what allows sailboats to move against the wind: high lift coefficients with low drag
//--------------------------------------------------------------------------------------------------------

function Angle2LiftDragForPaddle( const angle:Double; var cLift,cDrag:Double ):boolean;  // angle of attack in 0..90 range! in deg

implementation //----------------------------------------------------------------------------

uses
  System.Math, System.SysUtils;

// taken from an oar blade table
// C Oar blade lift and drag coefficients
// http://www.atkinsopht.com/row/liftdrag.htm
//     Hydrofoil: Flat plate, fully immersed, b/c=0.2, after Hoerner
const
  NumPointsLD=24;
//  Attack angle
//    0     1     2     3     4     5     6     7    8      9    10    11   12
//   13    14    15     16    17    18   19    20    21    22    23
  AttackAngle:Array [0..NumPointsLD-1] of Single=   // in degrees
   ( 0.0 , 1.0 , 2.0 , 3.0 ,  5.0 , 7.0 ,10.0 ,15.0 ,20.0 ,25.0 , 30.0 ,33.0,35.0 ,
    37.0 ,39.0 ,41.0 ,42.5 ,42.6,   45.0, 50.0, 60.0, 70.0, 80.0, 90.0);
  C_Lift_coefficient:Array [0..NumPointsLD-1] of Single=
   ( 0.0 , 0.03, 0.065,0.105,0.185,0.265,0.39 ,0.61 ,0.83 ,1.04 ,1.23 ,1.33,1.36 ,
    1.38 ,1.37 ,1.35 ,1.30 ,0.73, 0.71, 0.67, 0.56, 0.40, 0.21, 0.0   );
  C_Drag_coefficient:Array [0..NumPointsLD-1] of Single=
   ( 0.0 ,0.0005,0.001,0.005,0.015,0.03 ,0.07 ,0.17 ,0.31 ,0.49 ,0.71 ,0.84,0.93 ,
    1.01 ,1.08 ,1.15 ,1.20 ,0.66, 0.71, 0.80, 0.96, 1.09, 1.17, 1.20  );

//                       stol
// Coef ^                __
//      |              /  |
//      |            /    | Lift
//  1.0 +          /      |       ... Drag
//      |        /        \   .../
//      |      /         ..\./
//      |    /       ..../   \
//      |  /    ..../          \
//      |/...../                 \
//      +--------------------------\------>
//      0              45           90   Angle of attack

function Angle2LiftDragForPaddle(const angle:Double; var cLift,cDrag:Double ):boolean;    // angle in 0..90 range in deg
var a1,a2,cl1,cl2,cd1,cd2,deltaa,deltac:Double;
    L,H,I,Ix:Integer;
    aAngle,C:double;
begin
  if (angle<0) or (angle>90) then raise Exception.Create('Error: invalid angle of attack:'+Format('%5.3f',[angle]) );

  Result := false;

  L := 0;          // binary search angle (faster than linear)
  H := NumPointsLD-1;
  Ix:= 0;
  while (L<=H) do
    begin
      I := (L + H) shr 1;        //i=middle between H and L
      aAngle := AttackAngle[I];
      C:=( aAngle-angle);      // compare angles
      if (C<0) then L:=I+1    // C<0 --> angle é antes do apontado, vai pra metade de cima
        else begin           // C>=0 --> angle é dpois do apontado, vai pra metade de baixo
          if (C=0) then     // found exactly
            begin
              Ix:=I;
              break;
            end;
          H :=I-1;         //nop yet, save h
          Ix:=I;          //save last good index
        end;
    end;

   i:=Ix;  //retrieve index
   if (i=0) then begin cLift:=0; cDrag:=0; Result := true; exit;  end;

   a1  := AttackAngle[i-1];    //do linear interpolation between angles a[i] and a[i-1]
   a2  := AttackAngle[i];
   deltaa := a2-a1;

   cl1 := C_Lift_coefficient[i-1];
   cl2 := C_Lift_coefficient[i];
   deltac := cl2-cl1;

   if (deltaa<>0) then cLift := cl1+(angle-a1)*deltac/deltaa      //faz interpolacao linear
    else cLift:=cl1;                                              //se os ptos coincidem, pega i do pto antes

   cd1 := C_Drag_coefficient[i-1];
   cd2 := C_Drag_coefficient[i];
   deltac := cd2-cd1;

   if (deltaa<>0) then cDrag := cd1+(angle-a1)*deltac/deltaa      //faz interpolacao linear
    else cDrag:=cd1;                                              //se os ptos coincidem, pega i do pto antes

   Result := true;
end;

end.

