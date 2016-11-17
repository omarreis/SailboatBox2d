unit USailboatTest;  // TSailboat test
 //-------------//                                         v
//   by Omar. out16                                         v  v
//   a sail boat physics model for box2d                     v
//
// Om: Changed a few things in firebox2d master source:
//     1- enabled operation overload in Physics2D.inc . op ovl simplifies vector math a lot..
//     2- increased b2_maxPolygonVertices from 8 to 10 in UPhysics2DTypes.pas to draw my standard boat. Yeah  I know I could split..
//     3- added m_debugDrawer.DrawSmallText()

interface
{$I ..\..\Physics2D\Physics2D.inc}

{$DEFINE FIREMONKEYBOX2D}    // undef for VCL

uses
   System.UITypes,    //TColor
   {$IFDEF FIREMONKEYBOX2D}
   uTestBed,          //Firemonkey
   uDebugDrawerFM,   //TDebugDrawerFM
   uFMDrawUtils,    //RGBAToFMColor
   {$ELSE}
   UMain,             //VCL
   {$ENDIF FIREMONKEYBOX2D}
   // box2d
   UPhysics2DTypes,
   UPhysics2D,
   UPhysics2DHelper,
   UPhysics2DControllers,
   // /box2d
   uLiftDragCalculator,
   omSailboatHelper,    // sailboat related classes: TAutoPilot,..
   omSailboat,          // TSailboat

   Math, Classes, SysUtils;

{$DEFINE FIREMONKEYBOX2D}    // my own firemonkey/vcl switch

{$IFNDEF OP_OVERLOAD}
  ..... // not implemented!     this unit uses op overload extensively for vector math   :(
{$ENDIF OP_OVERLOAD}

{$IFDEF CONTROLLERS}

type
   TSailboatTest = class(TTester)
   private
      procedure CreateFreeRope;
      procedure ClearRopeSegments;
      procedure MakeRecicleWakeParticles;
      // Procedure SetRopeAsSail(aList:TList);
      procedure ApplyControlSlidersToBoat;
      procedure SetSliders(const jibTrim,mainTrim:double);  //trim 0 .. 1.0
      // all Values below in range 0 .. 1.0
      function  getMainSheetValue:double;
      function  getJibSheetValue:double;
      // rudder control. Value in 0..1   ( 0.5 = centered rudder )
      function  getRudderValue:double;
      procedure SetJibSheetValue(const Value:Double);
      procedure SetMainSheetValue(const Value:Double);
      procedure SetRudderValue(const Value:Double);
      // step() actions
      procedure ShowLiftDragTest;

      Procedure _DrawVectorArrow(const p1, p2: TVector2; const Text:String; const color: RGBA);
      procedure _DrawText(const text: string; const color: RGBA);

      Procedure  CenterCameraOnBoat;
      Procedure  AutoPilotActionExecute(const rudderSetting:Double);
   public
      // grounds
      m_groundJibSlider,m_groundMainSlider,m_groundRudderSlider,
      m_groundRope:Tb2Body;      //free rope flying

      bFollowBoat:boolean;       //with the camera

      m_Sailboat:TSailboat;   // test sailboat
      m_wake:TBoatWake;       // boat wake and sea waves (bubbles)
      m_TWD:double;           // true wind direction, in rads
      trueWind: TVector2;     // true wind vector, in m/s
      m_windController: TSailboatWindController;   // custom wind controller for sailboats

      freeRopeSegments: TOmSail;   // free rope flying near the boat

      m_island: Tb2Body;           // not shown
      m_WindIndicator: Tb2Body;    // wind direction indicator arrow
      m_testEdge: Tb2Body;         // edge shape for lift/drag calc testing

      // UI sliders, to control the sail sheets and boat rudder
      m_jibSheetSlider:Tb2Body;
      m_jibSliderJoint:Tb2PrismaticJoint;

      m_mainSheetSlider:Tb2Body;
      m_mainSliderJoint:Tb2PrismaticJoint;

      m_rudderSlider:Tb2Body;
      m_rudderSliderJoint:Tb2PrismaticJoint;

      constructor Create; override;
      destructor Destroy; override;
      procedure  Step(var settings: TSettings; timeStep: PhysicsFloat); override;
      procedure  Keyboard(key: Byte); override;
   end;
{$ENDIF}

implementation  //-------------------------------------------------

{$IFDEF CONTROLLERS}
const
   // This test was derioved from "ropes in wind". Kept a rope :)
   Rope: array[0..4] of TPointF = ( (x: -8; y: 10),  (x:-7; y: 9),  (x: -6; y: 8),  (x: -5; y: 9),  (x: -4; y: 10) );

var
   LateralSetinha : array[0..7] of TVector2 = (
     (x: 0   ;y: 0.5),                //   *0
     (x: 0.10;y:0.30),                //   |          <--seta
     (x: 0.10;y:0.30),                //   | *1=2 ??
     (x: 0.10;y:-0.5),                //   +----------
     (x:-0.10;y:-0.5),                //   |
     (x:-0.10;y:0.30),                //   | *3
     (x:-0.10;y:0.30),
     (x:-0   ;y: 0.5) );

   Island: array[0..3] of TVector2  = (       //not used..
     (x:  1.8; y: -2.1 ),
     (x: -2.0; y: -2.3 ),
     (x: -2.1; y:  1.9 ),
     (x:  2.3; y:  2.1 ) );

// wind controller ask the user for wind-force before every step. Modify this as you wish.
function WindCallback: TVector2;
begin
  Result := TSailboatTest(Test).m_Sailboat.apparentWind;
end;

{ TSailboatTest }

constructor TSailboatTest.Create;
var
   bd: Tb2BodyDef;
   polyShp : Tb2PolygonShape;
   edgeShp:Tb2EdgeShape;
   fix : Tb2FixtureDef;
   jp :Tb2PrismaticJointDef;

begin {TSailboatTest.Create}
   inherited;

   bShowVectors := false;

   bFollowBoat  := false;

   m_world.SetGravity(b2Vec2_Zero);     //no gravity
   UpdateGravityText;

   freeRopeSegments := TOmSail.Create('free rope');

   m_wake := TBoatWake.Create;

   maxseg := 0.65;    //maximum sail segment size

   m_windController := TSailboatWindController.Create;    //wind controller
   m_windController.Callback := WindCallback;            //get wind callbk
   m_world.AddController(m_windController);

   begin     //  create Island
      // polyShp := Tb2PolygonShape.Create;
      // polyShp.SetVertices( @Island[0], 4 );      // rectangular island

      bd          := Tb2BodyDef.Create;
      bd.bodyType := b2_staticBody;  //b2_dynamicBody;   //b2_kinematicBody; //
      m_island    := m_world.CreateBody(bd, true);

      // fix := Tb2FixtureDef.Create;
      // fix.shape := polyShp;
      // fix.filter.categoryBits := CAT_LAND;  // I'm a rock
      // fix.filter.maskBits     := MASK_ALL;  // land should colide with everything
      // fix.filter.groupIndex   := 0;         // 0 means use category

      //fix.restitution := 1.0;
      //fix.density     := 1000.0;

      // m_island.CreateFixture( fix, {AutoFreeFix=} true);
   end;


   // o cod comentado abaixo cria corpo tipo "retinha"
   // begin  //Om frexa do vento
   //    bd    := Tb2BodyDef.Create;
   //    SetValue(bd.position, 1.0, 15.0);
   //    bd.bodyType := b2_dynamicBody;
   //
   //    shape := Tb2EdgeShape.Create;
   //    shape.SetVertices( MakeVector(5,5),
   //                       MakeVector(10,10) );
   //
   //    vetorVento := m_world.CreateBody(bd, False);
   //    vetorVento.CreateFixture(shape, {density=}0.0, {AutoFreeShape=}False);
   //
   //    bd.Free;
   //    shape.Free;
   // end;

   begin  //create test edge for lift/drag
     bd    := Tb2BodyDef.Create;
     SetValue(bd.position, -10, 10.0);
     bd.bodyType := b2_kinematicBody;  // b2_dynamicBody;
     m_testEdge  := m_world.CreateBody(bd, false);

     edgeShp := Tb2EdgeShape.Create;
     edgeShp .SetVertices( MakeVector(-2,0), MakeVector( 2,0) );

     fix := Tb2FixtureDef.Create;           //attach rudder shape to body
     fix.shape := edgeShp;
     fix.filter.categoryBits := CAT_BOAT;   // I'm  a rudder, but secretly a boom. I must be inside the boat, so i cannot collide with it
     fix.filter.maskBits     := MASK_NONE;  // will !collide
     fix.filter.groupIndex   := 0;         // 0 means use category
     fix.restitution         := 0.5;
     fix.density             := 1.0;
     m_testEdge.CreateFixture(fix, {AutoFreeShape=} true);
   end;

   m_TWD := Pi/2;     // default wind-to = East

   begin     // floating arrow indicating wind direction                                   //      wind indicator
      bd    := Tb2BodyDef.Create;                                                         //               ___
      SetValue(bd.position, -10,0);                                                       //             /    |
      bd.bodyType :=  b2_dynamicBody; //b2_staticBody;                                    //            /     |
                                                                                          //           /     /
      polyShp := Tb2PolygonShape.Create;               // wind arrow indicator shape      //          /     /
      polyShp.SetVertices(@LateralSetinha[0], 8 );                                        //         .     /
                                                                                          //           .../
      m_WindIndicator := m_world.CreateBody(bd, true);                                    //

      fix := Tb2FixtureDef.Create;           //attach rudder shape to body
      fix.shape := polyShp;
      fix.filter.categoryBits := CAT_BOAT;   // I'm  a rudder, but secretly a boom. I must be inside the boat, so i cannot collide with it
      fix.filter.maskBits     := MASK_ALL;  // will collide with all
      fix.filter.groupIndex   := 0;          // 0 means use category
      fix.restitution         := 0.5;
      fix.density             := 1.0;
      m_WindIndicator.CreateFixture(fix,true);
      m_WindIndicator.SetTransform( m_WindIndicator.GetPosition , m_TWD-Pi );  //rotated ?
       //polyShp.Free;
      //bd.Free;
   end;


   // Prepare the stage to receive the sails, which are also ropes.. and also free rope, flying on the island
   ClearRopeSegments;
   CreateFreeRope;   // remained from ropes in wind, which was used as a basic for this
   m_Sailboat := TSailboat.Create(m_world, m_debugDrawer);
   //set debug drawing events
   m_Sailboat.m_OnDrawVector      := _DrawVectorArrow;
   m_Sailboat.m_OnDrawText        := _DrawText;
   m_Sailboat.m_OnAutoPilotAction := AutoPilotActionExecute;

   // begin // a bunch of wake particles
   //    // CreateWaterParticles;
   //    bd := Tb2BodyDef.Create;
   //    bd.bullet := true;
   //    bd.bodyType := b2_dynamicBody;
   //
   //    for i := 0 to 100 do
   //    begin
   //       bd.position := MakeVector(RandomFloat(-120, 40), RandomFloat(-30, 30) );
   //       bd.linearVelocity := MakeVector( RandomFloat(8.1,8), RandomFloat(-0.1, 0.1) );
   //       circleShp := Tb2CircleShape.Create;
   //       circleShp.m_radius := 0.01;
   //       aBubble     := m_world.CreateBody(bd, False);
   //       m_particles.Add(aBubble);
   //
   //       fix := Tb2FixtureDef.Create;           //attach rudder shape to body
   //       fix.shape := circleShp;
   //       fix.filter.categoryBits := CAT_LAND;   // I'm  a bubble, but secretly a boom.
   //       fix.filter.maskBits     := CAT_SAIL;  // will collide
   //       fix.filter.groupIndex   := 0;        // 0 means use category
   //       fix.restitution := 1.0;
   //       fix.density     := 0.1;
   //       aBubble.CreateFixture(fix,true);
   //    end;
   //    bd.Free;
   // end;

   m_windController.AddSail( m_Sailboat.jibSailSegments  );    // add sails to sailboat wind controller
   m_windController.AddSail( m_Sailboat.mainSailSegments );
   m_windController.AddSail( freeRopeSegments );

   freeRopeSegments.SetRopeAsSail;  // this sets sail segments attributes

   m_windController.AddSail( freeRopeSegments );
   freeRopeSegments.SetRopeAsSail;  // this sets sail segments attributes

   trueWind   := MakeVector( 10.29, 0);  //  20 kn=10.29 m/s   (no eixo x --> W wind)

   // create slider controls
   // Jib slider
   begin
       polyShp := Tb2PolygonShape.Create;
       polyShp.SetAsBox(1.0, 1.0);

       bd          := Tb2BodyDef.Create;
       bd.position := jibSliderInitPos;      // default jib slider pos
       bd.bodyType := b2_dynamicBody;
       //SetValue(bd.position, 0, 0);

       m_jibSheetSlider := m_world.CreateBody(bd, true);

       fix := Tb2FixtureDef.Create;           //attach mast shape to body
       fix.shape := polyShp;
       fix.filter.categoryBits := CAT_BOOM;      //  //??
       fix.filter.maskBits     := MASK_NONE;     // will not collide
       fix.filter.groupIndex   := 0;             // 0 means use category
       fix.restitution := 1.0;
       fix.density     := 1.0;
       m_jibSheetSlider.CreateFixture( fix,{AutoFreeFix=} true);

       bd          := Tb2BodyDef.Create;  //create a ground pt near the slider (used by the prismatic jt)
       bd.position := jibSliderInitPos;
       bd.bodyType := b2_staticBody;
       m_groundJibSlider := m_world.CreateBody(bd, true);  //no fixture

       jp := Tb2PrismaticJointDef.Create;
       jp.Initialize(m_groundJibSlider, m_jibSheetSlider, bd.position, MakeVector(0.0, 1.0));
       jp.lowerTranslation :=  0.0;
       jp.upperTranslation := 10.0;
       jp.enableLimit   := True;
       jp.enableMotor   := True;
       jp.maxMotorForce := 1000.0;
       jp.motorSpeed    := 0.0;
       m_jibSliderJoint := Tb2PrismaticJoint( m_world.CreateJoint(jp) );
   end;  // /Jib slider

   // Main slider
   begin
       polyShp := Tb2PolygonShape.Create;
       polyShp.SetAsBox(1.0, 1.0);

       bd          := Tb2BodyDef.Create;
       bd.position := MainSliderInitPos;      // default Main slider pos
       bd.bodyType := b2_dynamicBody;
       //SetValue(bd.position, 0, 0);

       m_mainSheetSlider := m_world.CreateBody(bd, true);

       fix := Tb2FixtureDef.Create;           //attach mast shape to body
       fix.shape := polyShp;
       fix.filter.categoryBits := CAT_BOOM;      //  //??
       fix.filter.maskBits     := MASK_NONE;     // will not collide
       fix.filter.groupIndex   := 0;             // 0 means use category
       fix.restitution := 1.0;
       fix.density     := 1.0;
       m_mainSheetSlider.CreateFixture( fix,{AutoFreeFix=} true);

       bd          := Tb2BodyDef.Create;  //create a ground pt near the slider (used by the prismatic jt)
       bd.position := MainSliderInitPos;
       bd.bodyType := b2_staticBody;
       m_groundMainSlider := m_world.CreateBody(bd, true);  //no fixture

       jp := Tb2PrismaticJointDef.Create; //joint slider to ground
       jp.Initialize(m_groundMainSlider, m_mainSheetSlider, bd.position, MakeVector(0.0, 1.0));
       jp.lowerTranslation :=  0.0;
       jp.upperTranslation := 10.0;
       jp.enableLimit   := True;
       jp.enableMotor   := True;
       jp.maxMotorForce := 1000.0;
       jp.motorSpeed    := 0.0;
       m_mainSliderJoint := Tb2PrismaticJoint( m_world.CreateJoint(jp) );
   end;  // /Main slider

   // Rudder slider
   begin
       polyShp := Tb2PolygonShape.Create;
       polyShp.SetAsBox(1.0, 1.0);

       bd          := Tb2BodyDef.Create;
       bd.position := RudderSliderInitPos+MakeVector(5,0);      // default Rudder slider pos
       bd.bodyType := b2_dynamicBody;
       //SetValue(bd.position, 0, 0);

       m_rudderSlider := m_world.CreateBody(bd, true);

       fix := Tb2FixtureDef.Create;           //attach mast shape to body
       fix.shape := polyShp;
       fix.filter.categoryBits := CAT_BOOM;      //
       fix.filter.maskBits     := MASK_NONE;     // will not collide
       fix.filter.groupIndex   := 0;             // 0 means use category
       fix.restitution := 1.0;
       fix.density     := 1.0;
       m_rudderSlider.CreateFixture( fix,{AutoFreeFix=} true);

       bd          := Tb2BodyDef.Create;  //create a ground pt near the slider (used by the prismatic jt)
       bd.position := RudderSliderInitPos+MakeVector(5,0);
       bd.bodyType := b2_staticBody;
       m_groundRudderSlider := m_world.CreateBody(bd, true);  //no fixture

       jp := Tb2PrismaticJointDef.Create; //joint slider to ground
       jp.Initialize(m_groundRudderSlider, m_rudderSlider, bd.position, MakeVector(1.0, 0.0));  // rudder is horizontal slider
       jp.lowerTranslation := -5.0;
       jp.upperTranslation := +5.0;
       jp.enableLimit   := True;
       jp.enableMotor   := True;
       jp.maxMotorForce := 1000.0;
       jp.motorSpeed    := 0.0;
       m_rudderSliderJoint := Tb2PrismaticJoint( m_world.CreateJoint(jp) );
   end;  // /Rudder slider

  SetSliders(0.15,0.25);            //set default sheet settings
end;

destructor TSailboatTest.Destroy;
begin
  freeRopeSegments.Free;

  m_windController.Free;

  m_wake.Free;

  m_Sailboat.Free;

  inherited;
end;

// draw vector
Procedure  TSailboatTest._DrawVectorArrow(const p1, p2: TVector2; const Text:String; const color: RGBA);
var a,b,p3,v:TVector2; L:Double;
begin
  b := p2-p1;
  L := b.Length;
  b.Normalize;
  a := MakeVector(-b.y,b.x );   // calc vec a, perpendicular to b
  {$IFDEF FIREMONKEYBOX2D}
  m_debugDrawer.DrawSegment(p1, p2, color);     // arrow shaft
  p3 := p2-(b+a)*L/30;
  m_debugDrawer.DrawSegment(p2, p3, color);     // arrow point left..
  p3 := p2-(b-a)*L/30;
  m_debugDrawer.DrawSegment(p2, p3, color);     // ..right

  //  v := p2-p1;
  //  S := Format('%4.0f', [v.Length] ) ;

  m_debugDrawer.DrawSmallText(p2,Text,color);

  {$ELSE}
  ..TODO
   m_debugDraw.DrawSegment(p1, p2, color);
  {$ENDIF FIREMONKEYBOX2D}
end;

// autopilot action handler
Procedure TSailboatTest.AutoPilotActionExecute(const rudderSetting:Double); // rudderSetting in 0 .. 1.0
begin
  SetRudderValue(rudderSetting);   //apply tyller setting
end;

Procedure TSailboatTest.CenterCameraOnBoat;
var bp:TVector2;
begin
  bp := m_Sailboat.getBoatPosition;  // boat pos in world coordinates
  {$IFDEF FIREMONKEYBOX2D}
  TDebugDrawerFM(m_debugDrawer).CenterOnWorldCoordinates(bp.x,bp.y);
  {$ELSE}
  ...// TODO
  {$ENDIF FIREMONKEYBOX2D}
end;

// make wake particles  (aka bubbles). recicle old
procedure TSailboatTest.MakeRecicleWakeParticles;
const xTEMPO=0.04/3600/24;

   function _RandVec(const Delta:Double):TVector2;
   begin   Result := MakeVector( RandomFloat(-Delta,Delta), RandomFloat(-Delta, Delta) );   end;

var bd :Tb2BodyDef;
    aBubble:Tb2Body;
    fix:Tb2FixtureDef;
    i,n:integer;
    aBubSpd,a,b,c:TVector2;
    aPolygon:Tb2PolygonShape;
    vertices: array[0..2] of TVector2; //for triang
    aBCR:TBubbleCreationRec;
begin
  // destroy old bubbles
  n:=0;       // number of wake particles recicled
  for i := m_wake.m_particles.Count-1 downto 0 do    // check ages
  begin
    aBubble := Tb2Body( m_wake.m_particles.Items[i] );
    aBCR    := TBubbleCreationRec(aBubble.UserData);  //retrieve creation rec
    if aBCR.Expired then         //finished
      begin
        m_world.DestroyBody(aBubble);
        m_wake.m_particles.Delete(i);
        {$IFDEF AUTOREFCOUNT}
        aBCR.__ObjRelease;      // i hate that automatic garbage collection
        {$ELSE}
        aBCR.Free;
        {$ENDIF AUTOREFCOUNT}
        inc(n);                 // count recicled
      end;
  end;
  // create new. one bubble each step
  if (m_wake.m_particles.Count<MAX_BUBBLES) and (Now-m_wake.lastBubbleCreated>xTEMPO) then  // c/ x tempo cria uma.. MaxParticles=300
    begin
      m_wake.lastBubbleCreated := Now;

      bd := Tb2BodyDef.Create;  //create def, just in case
      // bd.bullet   := true;     //needed ??
      bd.bodyType := b2_dynamicBody;
      aBCR := TBubbleCreationRec.Create; // save creation time. used to control life
      {$IFDEF AUTOREFCOUNT}
      aBCR.__ObjAddRef;            //disable gc
      {$ENDIF AUTOREFCOUNT}
      bd.userData := aBCR; // save creation record
      // make b a small vector pointing away from the boat's stern          //    .   .   .    .
      b := - m_Sailboat.getBoatSpeedVersor;                                 //   .    .   .   .
      // 3 different kinds of bubble make sea waves & boat wake             //     .    ^   .
      //  1) wake bubbles                                                   //    .     B  .  .
      //  2) sea wave bubbles                                               //   .   . /B\   .  .
      //  3) boat side wake (left and right)                                //     .  / V \    .
                                                                            //   .   /  :  \  .
      case Random(15) of  // some quantic effect while creating bubbles     //    . / . :   \   .
        0..7:  begin     // sea bubbles can pop anywhere in the view field
                 // random position around the boat
                 bd.position := m_Sailboat.getBoatPosition+MakeVector( RandomFloat(-20,20), RandomFloat(-30,30) );
                 aBubSpd := _RandVec(0.7);
               end;
        8..12: begin    //wake bubble. behind the boat
                 bd.position := m_Sailboat.m_rudder.GetPosition+2*b+_RandVec(0.7)  ; // place bubble in the back of the boat
                 aBubSpd := _RandVec(0.7);              // set bubble speed, away from the boat..
               end;                           // ..so normaly it should not hit it
        13:    begin    //righ side bow wake.
                 a := MakeVector(-b.y,b.x );   // calc vec a, perpendicular to b. a points to the side of the boat
                 c := m_Sailboat.getBoatPosition;
                 bd.position := c-b-3*a+_RandVec(0.25);    // this is like 45 deg to the side
                 aBubSpd     := -1.5*a-_RandVec(0.25);    //calc bubble speed, 45 deg away from the boat.. may be more..
               end;
        14:    begin    //left bow wake
                 a := MakeVector(-b.y,b.x );   //calc vec a, perpendicular to b
                 c := m_Sailboat.getBoatPosition;
                 bd.position := c-b+3*a+_RandVec(0.5);
                 aBubSpd     := 1.5*a+_RandVec(0.25); //calc bubble speed, away from the boat..
               end;
      else exit; //??
      end;

      bd.linearVelocity := aBubSpd;  // set bubble speed
      // mk a small triangle
      vertices[0] := MakeVector( RandomFloat(-0.25,-0.2),RandomFloat(0.0,0.2));   // ramdomic sizes
      vertices[1] := MakeVector( RandomFloat(0.2,0.25),  RandomFloat(0.0,0.2));
      vertices[2] := MakeVector( RandomFloat(0.0,0.1),   RandomFloat(0.3,0.5));
      aPolygon := Tb2PolygonShape.Create;
      aPolygon.SetVertices( @vertices[0], 3);     // this bubble is a little triangle..

      // circleShp := Tb2CircleShape.Create;
      // circleShp.m_radius := RandomFloat(0.1,0.2);
      aBubble     := m_world.CreateBody(bd, False);
      m_wake.m_particles.Add(aBubble);

      fix := Tb2FixtureDef.Create;           // bubble shape fixture
      fix.shape               := aPolygon;   //circleShp;
      fix.filter.categoryBits := CAT_BUBBLE; // I'm  a bubble
      fix.filter.maskBits     := CAT_BOAT or CAT_BUBBLE or CAT_LAND;    // I will collide generating kaos
      fix.filter.groupIndex   := 0;          // 0 here means "use category/mask"
      fix.restitution         := 0.5;
      fix.density             := 0.1;   //lite
      aBubble.CreateFixture(fix,true);
      bd.Free;
    end;
end;

function TSailboatTest.getJibSheetValue:double;  //from UI slider
begin
  Result := ( m_jibSheetSlider.GetPosition.y-jibSliderInitPos.y) /10;  // 0.0 --- 1.0
end;

procedure TSailboatTest.SetJibSheetValue(const Value:Double);
var yj:Double;
begin
  yj := jibSliderInitPos.y + Value*10;
  m_jibSheetSlider.SetTransform( MakeVector(m_jibSheetSlider.GetPosition.x,yj) , 0 );
end;

function TSailboatTest.getMainSheetValue:double;
begin
  Result := ( m_mainSheetSlider.GetPosition.y-MainSliderInitPos.y) /10;  // 0.0 --- 1.0
end;

procedure TSailboatTest.SetMainSheetValue(const Value:Double);  // Value in 0.0 --- 1.0
var ym:Double;
begin
  ym := mainSliderInitPos.y + Value*10;
  m_mainSheetSlider.SetTransform( MakeVector(m_mainSheetSlider.GetPosition.x,ym) , 0 );
end;

function TSailboatTest.getRudderValue:double;
begin
  Result := ( m_rudderSlider.GetPosition.x-rudderSliderInitPos.x) /10;  // 0.0 --- 1.0
end;

procedure TSailboatTest.SetRudderValue(const Value:Double);    // value in 0.0 --- 1.0
var xr:Double;
begin
  // DrawText( Format('rud V: %4.1f', [Value]) );
  xr := rudderSliderInitPos.x  + Value*10;
  m_rudderSlider.SetTransform( MakeVector(xr, m_rudderSlider.GetPosition.y) , 0 );
end;

// get slider positions and set sheet lenghts accordingly
procedure TSailboatTest.ApplyControlSlidersToBoat;
var sj,sm,sr:Double;
begin
  sj := getJibSheetValue;      // control settings in 0..1 range
  sm := getMainSheetValue;
  sr := getRudderValue;

  DrawText( Format('Ctrl jib:%4.01f main:%4.0f rud:%4.0f',[sj*100,sm*100,sr*100])  );
  m_Sailboat.ApplyControlSettings( sj, sm, sr );   //apply UI to boat
end;

procedure TSailboatTest.SetSliders(const jibTrim,mainTrim:double);  //trim 0 .. 1.0
begin
  SetJibSheetValue(jibTrim);
  SetMainSheetValue(mainTrim);
end;

// this shows a test edge response to wind ( L/D model )
procedure TSailboatTest.ShowLiftDragTest;
// var sumF,p1,p2:TVector2; shape:Tb2EdgeShape;  xf: Tb2Transform;
begin
   // shape := Tb2EdgeShape( m_testEdge.GetFixtureList.GetShape );
   // xf    := m_testEdge.GetTransform^;
   // sumF := b2Vec2_Zero;
   // // compute two sides of the sail. Only the side pointed to the wind responds
   // // hacked original _ComputeSegmentEffectiveForce to use lift/drag model
   // // use apparent wind to fill the sail
   // _ComputeSegmentEffectiveForceLD( shape.m_vertex1, shape.m_vertex2, shape.m_normal1, xf, apparentWind,{RoFluid=}0.8, sumF );
   // // DrawText(Format('Lift: %4.1f Drag: %4.1f',[sumF.y,sumF.x]) );
   // _ComputeSegmentEffectiveForceLD( shape.m_vertex2, shape.m_vertex1, shape.m_normal2, xf, apparentWind,{RoFluid=}0.8, sumF );
   //
   // DrawText(Format('Lift: %4.1f Drag: %4.1f',[sumF.y,sumF.x]) );
   //
   // sumF := sumF*FORCE2SCREEN_SCALE;  // ad hoc scale factor to
   // p1 := m_testEdge.GetPosition;
   // p2 := p1+sumF;
   // _DrawVectorArrow(p1, p2, colorBlu);     // draw sail chord (blue)
end;

procedure TSailboatTest.Step(var settings: TSettings; timeStep: PhysicsFloat);
begin
   m_Sailboat.updateWindVectors(trueWind);  //update boat's wind vectors before calling m_World.Step()..
   ApplyControlSlidersToBoat;    // update sail trim with input from track bars (aka sliders)

   // ..so that the wind controller has the correct apparent wind
   inherited;         // this calls m_windcontroller.Step() which applies forces to sail segments

   if bFollowBoat then CenterCameraOnBoat;  // keeps jumping ??

   //if bShowParticles then
   MakeRecicleWakeParticles;     // boat wake simulated by sml floating triangles

   // ShowLiftDragTest;

   //show keyboard help
   begin
     DrawText('ASDW wind  JK-jibSheet / NM-outhaul');
     DrawText('UI mainSheet ZX-SpeedVec  FG-rudder');
     DrawText('Q release  . follow  , auto');
   end;

   m_Sailboat.Step( settings, timeStep); //boat step

   // p1 := MakeVector(  0,100);
   // p2 := MakeVector(100,100);
   // _DrawVectorArrow(p1,p2, colorVerde);

   // DrawText(Format('App W: (X: %f, Y: %f) angle: %4.0f', [apparentWind.x, apparentWind.y, -m_TWD*180/Pi] ));

   //DrawText(Format('Jib Sheet Len: %4.1f', [ m_jibSheet.MaxLength]));

   NextLine;
   //DrawText('Note that too smooth or too large wind force may break the rope.');
end;

procedure TSailboatTest._DrawText(const text: string; const color: RGBA);
var aColor:TColor;
begin
  aColor := RGBAToFMColor(color);
  m_debugDrawer.SetDefaultFontColor(aColor);
  DrawText(text);
end;

procedure TSailboatTest.Keyboard(key: Byte);
var L,a,Cap:double;
begin
   inherited;
   case key of
      187{+}:
         if maxseg < 4.0 then
         begin
            maxseg := maxseg + 0.2;
            CreateFreeRope;
         end;
      189{-}:
         if maxseg > 0.6 then
         begin
            maxseg := maxseg - 0.2;
            CreateFreeRope;
         end;

      Ord('A'),
      Ord('S'),
      Ord('D'),
      Ord('W'): begin
                  L := trueWind.Length;
                  a := ArcTan2( trueWind.x, -trueWind.y);
                  case Key of
                    Ord('A'): L:=L/1.05;
                    Ord('D'): L:=L*1.05;
                    Ord('W'): a:=a+Pi/180;
                    Ord('S'): a:=a-Pi/180;
                  end;
                  trueWind.x :=  L*Sin(a);
                  trueWind.y := -L*Cos(a);
                  m_Sailboat.updateWindVectors(trueWind);
                  m_TWD := a;  // windAngle (true wind)
                  m_WindIndicator.SetTransform( m_WindIndicator.GetPosition , m_TWD-Pi);    //rotated ?
                end;
      Ord('Z'): if m_Sailboat.boatSpeed<20    then
                begin
                  m_Sailboat.boatSpeed := m_Sailboat.boatSpeed+0.5;
                  m_Sailboat.updateWindVectors(trueWind);
                end;
      Ord('X'): if m_Sailboat.boatSpeed>0   then
                begin
                  m_Sailboat.boatSpeed := m_Sailboat.boatSpeed-0.5;
                  m_Sailboat.updateWindVectors(trueWind);
                end;

      // sheet trimming
      Ord('J'): begin
                   L := getJibSheetValue;
                   if (L<0.98) then
                     begin
                       L:=L+0.02;  //inc
                       SetJibSheetValue(L);
                     end;
                end;
      Ord('K'): begin
                   L := getJibSheetValue;
                   if (L>0.02) then
                     begin
                       L:=L-0.02;  //dec
                       SetJibSheetValue(L);
                     end;
                end;

      Ord('N'): if m_Sailboat.m_mainOutHaul.Length<2   then
                   m_Sailboat.m_mainOutHaul.Length  := m_Sailboat.m_mainOutHaul.Length +0.1;
      Ord('M'): if m_Sailboat.m_mainOutHaul.Length>0   then
                   m_Sailboat.m_mainOutHaul.Length  := m_Sailboat.m_mainOutHaul.Length -0.1;

      Ord('U'): begin
                   L := getMainSheetValue;
                   if (L<0.95) then
                     begin
                       L:=L+0.05;  //inc
                       SetMainSheetValue(L);
                     end;
                end;
      Ord('I'): begin
                   L := getMainSheetValue;
                   if (L>0.05) then
                     begin
                       L:=L-0.05;  //dec
                       SetMainSheetValue(L);
                     end;
                end;
      Ord('V'): bShowVectors := not bShowVectors;  //toggle
      Ord('*'): begin
                  maxseg := maxseg+0.4;
                  if maxseg>6.0 then maxseg:=0.6;
                  // TODO: rebuild rope ??
                end;

      Ord('R'): m_testEdge.SetTransform( m_testEdge.GetPosition, m_testEdge.GetTransform.q.GetAngle+Pi/180);
      Ord('T'): m_testEdge.SetTransform( m_testEdge.GetPosition, m_testEdge.GetTransform.q.GetAngle-Pi/180);
      Ord('L'): bUseLDformula := not bUseLDformula;
      Ord('Q'): begin
                  if Assigned(m_Sailboat.m_boatGroundJt) then  //release boat by destroing joint
                     begin
                       m_world.DestroyJoint(m_Sailboat.m_boatGroundJt);
                       m_Sailboat.m_boatGroundJt := nil;
                     end;
                end;
      Ord('.'): bFollowBoat := not bFollowBoat; //toggle floow boat
      Ord('/'): begin
                   if (e_jointBit in m_debugDrawer.m_drawFlags) then
                     Exclude(m_debugDrawer.m_drawFlags,e_jointBit)
                     else Include(m_debugDrawer.m_drawFlags,e_jointBit);
                end;
      Ord('G'): begin
                   if m_Sailboat.m_AutoPilot.Enabled then
                     begin
                       Cap := m_Sailboat.m_AutoPilot.TargetCourse+1;
                       if (Cap>=360) then Cap := Cap-360;
                       m_Sailboat.m_AutoPilot.SetCourse(Cap);   // inc autopilot target course
                     end
                     else begin
                       L := getRudderValue;
                       if (L<0.98) then
                         begin
                           L:=L+0.01;  //inc
                           SetRudderValue(L);
                         end;
                     end;
                end;
      Ord('F'): begin
                   if m_Sailboat.m_AutoPilot.Enabled then
                     begin
                       Cap := m_Sailboat.m_AutoPilot.TargetCourse-1;
                       if (Cap<0) then Cap := Cap+360;
                       m_Sailboat.m_AutoPilot.SetCourse(Cap);   // dec autopilot target course
                     end
                     else begin
                       L := getRudderValue;  // manually rotate rudder
                       if (L>0.02) then
                         begin
                           L:=L-0.01;  //dec
                           SetRudderValue(L);
                         end;
                     end;
                end;

      Ord(','): begin
                  m_Sailboat.m_AutoPilot.Enabled := not m_Sailboat.m_AutoPilot.Enabled;  //toggle AP
                  if m_Sailboat.m_AutoPilot.Enabled then
                     begin
                        m_Sailboat.m_AutoPilot.SetCourse(m_Sailboat.boatCourse);   // save current course as target
                     end;
                end;
      // rope joint version
      // Ord('J'): if m_jibSheet.MaxLength<6 then m_jibSheet.MaxLength := m_jibSheet.MaxLength+1;
      // Ord('K'): if m_jibSheet.MaxLength>0 then m_jibSheet.MaxLength := m_jibSheet.MaxLength-1;
      // Ord('N'): if m_MainOutHaul.MaxLength<4 then m_MainOutHaul.MaxLength := m_MainOutHaul.MaxLength+1;
      // Ord('M'): if m_MainOutHaul.MaxLength>0 then m_MainOutHaul.MaxLength := m_MainOutHaul.MaxLength-1;
   end;
end;

procedure TSailboatTest.ClearRopeSegments;  //and wind controller elements
var i: Integer;
begin
   // m_Sailboat.ClearRopeSegments;
   for i := 0 to freeRopeSegments.Count - 1 do m_world.DestroyBody(Tb2Body(freeRopeSegments[i]));
   freeRopeSegments.Clear;
end;

procedure TSailboatTest.CreateFreeRope;
var bd :Tb2BodyDef;
begin
  bd           := Tb2BodyDef.Create;  //create a ground pt near the slider (used by the prismatic jt)
  bd.position  := MakeVector(Rope[0].x,Rope[0].y);
  bd.bodyType  := b2_staticBody;
  m_groundRope := m_world.CreateBody(bd, true);  //no fixture

  BuildRope(@Rope[0], 5, m_world, freeRopeSegments, m_groundRope, nil, 0.1, maxseg);
end;

//----------------------------------------

Procedure ScaleLateralSetinha;
var i:integer;
begin
  for i := 0 to 7 do
    begin LateralSetinha[i].x := LateralSetinha[i].x*10; LateralSetinha[i].y := LateralSetinha[i].y*10; end;
end;

initialization
   ScaleLateralSetinha;
   RegisterTestEntry('aa_Sailboat', TSailboatTest);

{$ENDIF}
end.

