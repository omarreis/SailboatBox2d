unit omSailboat;   // TSailboat object.
 //-------------//                                         v
//   by Omar. out16                                         v  v
//   a sail boat physics model for box2d                     v
//
//                : :                     V
//               / \   :                    V    V               x         xx
//              /   \    :                    V                   Xx     XX
//             /     \    :                                        Xxx xX
//            /       \    :  1-flapping ajustable Main sail..     xxXXxX Xxxx
//           /   O:    \     2-..and Jib sail                    xxx _xIIXx
//          /    I  :   \     3-main boom                       x  |_|     x
//         |      I  :   |     4-rudder                           |_|
//         |       I  :  |     5-auto pilot                      |_|
//         |        I  : |              |                        |_|
//          \     |     /               |\                 /-----|_|---\
//           \----|----/             ___|=________________/     /---\   \_____
//                |
//

interface
{$I ..\..\Physics2D\Physics2D.inc}

{$DEFINE FIREMONKEYBOX2D}    // undef for VCL

uses
   Classes, SysUtils, Math,
   {$IFDEF FIREMONKEYBOX2D}
   uTestBed,         //Firemonkey
   uDebugDrawer,
   uDebugDrawerFM,   //TDebugDrawerFM
   {$ELSE}
   UMain,            //VCL
   {$ENDIF FIREMONKEYBOX2D}
   // box2d
   UPhysics2DTypes,
   UPhysics2D,
   UPhysics2DHelper,
   UPhysics2DControllers,
   // /box2d
   uLiftDragCalculator,   // lift/drag calculator for a slender,flat plate
   omSailboatHelper;      // sailboat related classes

{$IFNDEF OP_OVERLOAD}     //needs operantor overload for vector math   :
  ....// not implemented!
{$ENDIF OP_OVERLOAD}


const  //initial positions
  boatInitPos:TVector2         = (X: 5.0;  Y: 0.0 );
  jibSliderInitPos :TVector2   = (X: 7.0;  Y:10.0 );
  mainSliderInitPos:TVector2   = (X: 11.0; Y:10.0 );
  rudderSliderInitPos:TVector2 = (X:  3.0; Y:20.0 );  //TODO acertar
  //main and jib sheet limits
  MIN_JIB_SHEET_LEN =1.3;   MAX_JIB_SHEET_LEN =11.0;  //min/max do jt.len
  MIN_MAIN_SHEET_LEN=0.2;   MAX_MAIN_SHEET_LEN=6.0;
  MINMAX_RUDDER_ANG=Pi/4;   // +-45 deg

  FORCE2SCREEN_SCALE=1/50;  // scale force magnitude vectors to a suitable screen size
  SPEED2SCREEN_SCALE=1/50;

  // custom colors used for debugdraw vectors
  colorBlu:   RGBA = (0.3, 0.3, 0.8,  1.0);  // navy
  colorVerde: RGBA = (0.1, 0.8, 0.7,  1.0);  // more like cian
  colorRosso: RGBA = (1.0, 0.1, 0.5,  1.0);  // this should be the wine
  colorWine:  RGBA = (0.9, 0.3, 0.0,  1.0);  // not a very good one..
  colorAppW:  RGBA = (0.6, 0.4, 0.25, 1.0);  // apparent wind color. brown..
  colorCINZA: RGBA = (0.2, 0.2, 0.52, 1.0);
  colorHDrag: RGBA = (0.3, 0.7, 0.52, 1.0);

type
  // debug drawing events
  TDrawVectorEvent=procedure(const p1, p2: TVector2; const Text:String; const color: RGBA) of object;  // p1,p2 real world coords
  TDrawTextEvent  =procedure(const text: string; const color: RGBA)     of object;
  TAutoPilotAction=procedure(const rudderSetting:Double)                of object;

  /// TSailboat class - implements physics and rigging of a 2D sailboat
  TSailboat=class
  private
    function   getBoatSpeedVec: TVector2;
    function   getKeelAngleOfAttack: double;
    function   getBoatForwardVelocity: TVector2;
    function   getBoatLateralVelocity: TVector2;
    //debug draw funcs
    Procedure  _DrawVectorArrow(const p1, p2: TVector2; const Text:String; const color: RGBA);
    procedure  _DrawText(const text: string; const color: RGBA);
  public
    //events to communicate with test form
    m_OnDrawVector:TDrawVectorEvent;
    m_OnDrawText: TDrawTextEvent;
    m_OnAutoPilotAction:TAutoPilotAction;
    // box2d objects
    m_world:Tb2World;
    m_debugDrawer: TDebugDrawer;
    m_groundBoat:Tb2Body;  //boat ground used when boat is "anchored"

    //boat parts
    m_boatHull: Tb2Body;
    m_mast: Tb2Body;
    m_boom: Tb2Body;
    m_rudder: Tb2Body;
    //sails
    m_jibSail: Tb2Body;     // last segment (edge) of the sail. Used to joint the sail chain to its sheet
    m_mainSail: Tb2Body;

    jibSailSegments : TOmSail; // list of T2bBody s ( like ropes, sails are a bunch of edge shapes connected by revoluteJoints )
    mainSailSegments: TOmSail;

    // instrument vars
    boatSpeed: double;        //over ground
    boatCourse: double;
    TWS,TWD:Double;   //true wind
    AWS,AWD:Double;   //apparent wind
    TWA:Double;       //true wind angle, used
    AWA:Double;

    //sheets
    m_jibSheet : Tb2RopeJoint;           // Tb2DistanceJoint or Tb2RopeJoint ??
    m_mainOutHaul:Tb2DistanceJoint;      // esteira
    m_mainSheet: Tb2RopeJoint;           // Main sheet joint
    m_rudderCtrlJt: Tb2RevoluteJoint;    // rudder ctrl joint
    m_jibTravellerJt:Tb2PrismaticJoint;  //jib treveller
    m_jibTravellerCar:Tb2Body;           //rectangular jib car

    m_boatGroundJt:Tb2RevoluteJoint;     //optional, to keep the boat anchored. Click Q to release

    m_jibForceAvg :TExponentialMovingAverageVector;   // force average calculators. Not used right now
    m_mainForceAvg:TExponentialMovingAverageVector;

    m_AutoPilot:TAutoPilot;              //auto steer

    //wind vectors
    trueWind: TVector2;            // TWS
    apparentWind: TVector2;       // AWS  -  calculated from TW and BS

    constructor Create(aWorld:Tb2World; adebugDrawer:TDebugDrawer);
    destructor  Destroy; override;
    procedure   ClearRopeSegments;
    procedure   ShowForceInSail(aSailSegments:TOmSail; timeStep: PhysicsFloat);
    procedure   ApplyControlSettings(const jibSett,mainSett,rudderSet:Double);  // settings in range 0..1.0
    procedure   ApplyForcesToBoat;    // rudder forces, keel forces and some cheats
    procedure   ShowBoatCourseSpeed;  //and speed
    procedure   UpdateInstruments(timeStep: PhysicsFloat);
    Procedure   getSailChordCamber(const aSailname:String; aList:TList; var aChord,aCamber,alfa:Double);  //aList is a TList of Tb2Body s
    procedure   Step(var settings:TSettings; timeStep: PhysicsFloat);
    procedure   updateWindVectors(const aTrueWind:TVector2);
    function    getBoatPosition: TVector2;
    function    getBoatSpeedVersor:TVector2;
  end;

implementation //------------------------------------------------------------

// uses PointToLine;

var
   LateralBarco: array[0..9] of TVector2 = (                // boat hull (boat 10m long)
    (x:  0   ;y: 0.5  ),
    (x:  0.15;y: 0.25 ),
    (x:  0.20;y: 0.0  ),
    (x:  0.19;y: -0.25),
    (x:  0.16;y: -0.5 ),
    (x: -0.16;y: -0.5 ),
    (x: -0.19;y: -0.25),
    (x: -0.20;y: 0.0  ),
    (x: -0.15;y: 0.25 ),
    (x: -0   ;y: 0.5  ) );  // *10 = 5.0

   VelaJib: array[0..4] of TPointF = (   //   +     0.0,5.0 = bow
     (x:  0   ;y:  0.5  ),               //    \
     (x:  0.1 ;y:  0.35 ),               //     \
     (x:  0.12;y:  0.2  ),               //     |
     (x:  0.13;y:  0.05 ),               //     |
     (x:  0.14;y: -0.1  ) );             //    /

   VelaMain: array[0..5] of TPointF = (       //   O   <-- 0,0
     (x:  0   ;y:  0.0 ),                     //    \
     (x:  0.1 ;y: -0.1 ),                     //     \
     (x:  0.12;y: -0.2 ),                     //     |
     (x:  0.13;y: -0.3 ),                     //     |
     (x:  0.13;y: -0.4 ),                     //     |
     (x:  0.13;y: -0.5 ) );                   //     |

function _AngleBetweenVectors(const v1,v2:TVector2):double;    //..with signal, in rads
begin
  Result := Math.ArcTan2(v2.y,v2.x) - Math.ArcTan2(v1.y,v1.x);  //from http://www.euclideanspace.com/maths/algebra/vectors/angleBetween/
  if (Result<=-Pi)     then Result:=Result+2*Pi                //put in the -pi..+pi range
  else if (Result>+Pi) then Result:=Result-2*Pi;
end;

// segment v1-v2 is considered a flat plate. Returns Lift+Drag of the foil in F
procedure CalcLiftDragWaterfoil(const v1,v2,aBoatSpeed:TVector2; const xf: Tb2Transform; var F:TVector2 );
var normal:TVector2;
const RoWater=1000.0;  // water density 1000kg/m3
begin
   // compute two sides of the sail. Only the side pointed to the wind responds
   // hacked _ComputeSegmentEffectiveForce to use lift/drag model
   F := b2Vec2_Zero;

   // redundant code ahead: Compute lift/drag for both sides of the foil, but it skips the face looking back
   normal := b2Cross(v2 - v1, 1.0);
   normal.Normalize;
   _ComputeSegmentEffectiveForceLD( v1, v2, normal, xf, -aBoatSpeed , RoWater, F ); // side 1  lift/drag

   normal := -normal;    // other normal
   _ComputeSegmentEffectiveForceLD( v2, v1, normal, xf, -aBoatSpeed, RoWater ,F );  //side 2

   //DrawText(Format('Lift: %4.1f Drag: %4.1f',[sumF.y,sumF.x]) );
end;

{ TSailboat }

constructor TSailboat.Create(aWorld:Tb2World; adebugDrawer:TDebugDrawer);
var
   i: Integer;
   bd: Tb2BodyDef;
   box : Tb2PolygonShape;
   jdst : Tb2DistanceJointDef;
   polyShp : Tb2PolygonShape;
   circleShp: Tb2CircleShape;
   edgeShp:Tb2EdgeShape;
   aVela: array[0..5] of TPointF;
   Proa,BoatCenter,e:TVector2;
   fix : Tb2FixtureDef;
   rjd:Tb2RopeJointDef;
   revj:Tb2RevoluteJointDef;
   wjd:Tb2WeldJointDef;
   jp :Tb2PrismaticJointDef;
   vertices: array[0..9] of TVector2;
   fx :Tb2Fixture;

begin
  inherited Create;
  m_OnDrawVector:=nil;
  m_OnDrawText  :=nil;
  m_OnAutoPilotAction:=nil;

  m_World       := aWorld;        // not owned
  m_debugDrawer := adebugDrawer;  // not owned

  // moving average of force vectors
  m_jibForceAvg  := TExponentialMovingAverageVector.Create({numPer=}60);
  m_mainForceAvg := TExponentialMovingAverageVector.Create({numPer=}60);

  jibSailSegments  := TOmSail.Create('Jib');
  mainSailSegments := TOmSail.Create('Main');

  // Create b2d objects

  // Boat hull
   begin
      polyShp := Tb2PolygonShape.Create;
      polyShp.SetVertices( @LateralBarco[0], 10 );  //boat hull shape

      bd          := Tb2BodyDef.Create;
      bd.position := boatInitPos;
      bd.bodyType := b2_dynamicBody;

      m_boatHull := m_world.CreateBody(bd, true);

      fix := Tb2FixtureDef.Create;
      fix.shape := polyShp;
      fix.filter.categoryBits := CAT_BOAT;                 // I'm a boat
      fix.filter.maskBits     := MASK_NO_SAIL_MAST_BOOM;  // hull will not collide with sail, mast & boom, so these can be inside
      fix.filter.groupIndex   := 0;                      // 0 means use category
      fix.restitution := 0.3;
      fix.density     := 40;       //boat density   approx (10*3) 30 m2 and weight=1200Kg --> density=40 Kg/m2
      m_boatHull.CreateFixture(fix,true);
   end;

   // mast
   begin
      circleShp := Tb2CircleShape.Create;
      circleShp.m_radius := 0.3;
      bd          := Tb2BodyDef.Create;
      bd.position := m_boatHull.GetPosition+MakeVector(0,1);     //mast a little in front of the boat center (1m)
      bd.bodyType := b2_dynamicBody;
      //SetValue(bd.position, 0, 0);
      m_mast := m_world.CreateBody(bd, true );

      fix := Tb2FixtureDef.Create;           //attach mast shape to body
      fix.shape := circleShp;
      fix.filter.categoryBits := CAT_MAST;        // I'm a mast.
      fix.filter.maskBits     := MASK_NO_BOAT;   // mast will not collide with boat hull
      fix.filter.groupIndex   := 0;             // 0 means use category
      fix.restitution := 1.0;
      fix.density     := 100.0;
      m_mast.CreateFixture(fix,{AutoFreeFix=} true);

      //joint mast to boat
      wjd := Tb2WeldJointDef.Create;
      wjd.Initialize(m_boatHull, m_mast, m_mast.GetPosition);

      // revj := Tb2RevoluteJointDef.Create;    //rotating mast?  nah
      // revj.lowerAngle := 0.0;
      // revj.upperAngle := 2*Pi;
      // revj.Initialize( m_boatHull, m_mast, m_mast.GetPosition );
      // revj.collideConnected := false;
      m_world.CreateJoint(wjd,true);
   end;

   // boom
   begin
      vertices[0] := MakeVector( 0.1, 0.0);  //rect
      vertices[1] := MakeVector( 0.1,-5.5);
      vertices[2] := MakeVector(-0.1,-5.5);
      vertices[3] := MakeVector(-0.1, 0.0);
      box := Tb2PolygonShape.Create;
      box.SetVertices(@vertices[0],4);

      bd          := Tb2BodyDef.Create;
      bd.bodyType := b2_dynamicBody;
      bd.position := m_boatHull.GetPosition+MakeVector(0,0.7);    // mast a little in front of the boat center
      //SetValue(bd.position, 0, 0);
      m_boom := m_world.CreateBody(bd,true );

      fix := Tb2FixtureDef.Create;           //attach boom shape to body
      fix.shape := box;
      fix.filter.categoryBits := CAT_BOOM;   // I'm  boom. I must be inside the boat, so i cannot collide with it
      fix.filter.maskBits     := MASK_NONE;  // will collide with nothing
      fix.filter.groupIndex   := 0;          // 0 means use category
      fix.restitution := 1.0;
      fix.density     := 10.0;
      m_boom.CreateFixture(fix,true);

      revj := Tb2RevoluteJointDef.Create;  //joint boom to mast
      revj.lowerAngle := -Pi/2;           // boom has a 90 deg range
      revj.upperAngle :=  Pi/2;
      revj.enableLimit:= true;

      revj.Initialize(m_mast, m_boom, m_boom.GetPosition );
      revj.collideConnected := false;
      m_world.CreateJoint(revj,true);
   end;

   // rudder
   begin
      vertices[0] := MakeVector( 0.05, 2.0);  // 2m tiller
      vertices[1] := MakeVector( 0.05,-0.7);  // rudder blade 0.7m
      vertices[2] := MakeVector(-0.05,-0.7);
      vertices[3] := MakeVector(-0.05, 2.0);
      box := Tb2PolygonShape.Create;
      box.SetVertices(@vertices[0],4);

      bd          := Tb2BodyDef.Create;
      bd.bodyType := b2_dynamicBody;
      bd.position := m_boatHull.GetPosition+MakeVector(0, -5);     // rudder joint pos
      //SetValue(bd.position, 0, 0);
      m_rudder := m_world.CreateBody(bd,true );

      fix := Tb2FixtureDef.Create;           //attach rudder shape to body
      fix.shape := box;
      fix.filter.categoryBits := CAT_BOOM;   // I'm  a rudder, but secretly a boom. I must be inside the boat, so i cannot collide with it
      fix.filter.maskBits     := MASK_NONE;  // will collide with nothing
      fix.filter.groupIndex   := 0;          // 0 means use category
      fix.restitution := 1.0;
      fix.density     := 30.0;  //make rudder a little heavy
      m_rudder.CreateFixture(fix,true);

      revj := Tb2RevoluteJointDef.Create;   //joint rudder to boat by a revolute jt
      revj.Initialize( m_boatHull, m_rudder, m_rudder.GetPosition+MakeVector(0,-0.1 ));    //joint 0.1 outside
      revj.lowerAngle := -Pi/4;           //    rudder has +/- 45 deg range
      revj.upperAngle := +Pi/4;         //
      revj.enableLimit := true;

      revj.motorSpeed     := 0;
      revj.maxMotorTorque := 100.0;  // take care with this. Too much torque makes the boat shake
      revj.enableMotor    := true;
      revj.referenceAngle := 0;    //what is this?
      revj.collideConnected := false;
      m_rudderCtrlJt := Tb2RevoluteJoint( m_world.CreateJoint(revj,true) );
   end;

   begin  //create sails
      // Create sails with the boat in final position.
      // I found that creating sails at (0,0) and then teletransporting the boat tears up the sails !

      BoatCenter := m_boatHull.GetPosition;
      Proa       := BoatCenter+MakeVector(0,5);   // Proa=bow

      begin  // Jib sail ---------------------------------------------
        for i:=0 to 4 do    //translate sail def points to boat position, cause BuildRope uses absolute position
          aVela[i].SetValue(VelaJib[i].x+BoatCenter.x,VelaJib[i].y+BoatCenter.y);   //calc posição dos ptos da vela
        // create sail
        BuildRope( @aVela[0], 5, m_world, jibSailSegments, m_boatHull, nil, 0.1, maxseg );
        m_jibSail := Tb2Body(jibSailSegments.Last);        // save jib ptr

        // traveller system for the Jib
        polyShp := Tb2PolygonShape.Create;     // jib traveller car is a square
        polyShp.SetAsBox(0.2, 0.2);

        bd           := Tb2BodyDef.Create;  // jib traveller car
        bd.position  := BoatCenter+MakeVector(0,-2.5);     //same placeas the boat, so the jt is invisible
        bd.bodyType  := b2_DynamicBody;
        m_jibTravellerCar := m_world.CreateBody(bd, true);  //no fixture

        fix := Tb2FixtureDef.Create;           //attach car shape to body
        fix.shape := polyShp;
        fix.filter.categoryBits := CAT_BOOM;   // I'm secretly a boom. I must be inside the boat, so i cannot collide with it
        fix.filter.maskBits     := MASK_NONE;  // will collide with nada
        fix.filter.groupIndex   := 0;          // 0 means use category
        fix.restitution         := 0.5;
        fix.density             := 1.0;
        m_jibTravellerCar.CreateFixture(fix,true);

        jp := Tb2PrismaticJointDef.Create;   //jib traveller is a prismatic jt
        jp.Initialize(m_boatHull, m_jibTravellerCar, m_jibTravellerCar.GetPosition , {x axis}MakeVector(1.0, 0.0));
        jp.lowerTranslation := -1.6;  // jib traveller range
        jp.upperTranslation := +1.6;
        jp.enableLimit   := True;
        jp.enableMotor   := True;
        jp.maxMotorForce := 10.0;
        jp.motorSpeed    := 0.0;
        m_jibTravellerJt := Tb2PrismaticJoint( m_world.CreateJoint(jp) );  //jib treveller
        // /traveller system for the Jib

        // at first I wanted to use a rope joint as jib sheet, but this didn't work.
        // Opted for a dist joint, which is cheating I guess
        // use a distance joint (a stiff pole) or..
        // jdst := Tb2DistanceJointDef.Create;
        // jdst.Initialize(m_jibTravellerCar, m_jibSail, m_jibTravellerCar.GetPosition , m_jibSail.GetWorldCenter);
        // no motors in the dist jt ??
        // m_jibSheet := Tb2DistanceJoint( m_world.CreateJoint(jdst, true) );

        // .. use a rope joint.. didn't work..at first..
        rjd       := Tb2RopeJointDef.Create;
        rjd.bodyA := m_jibTravellerCar;
        rjd.bodyB := m_jibSail;  // m_jibSail points last seg of the sail
        rjd.localAnchorA := MakeVector(0,0  );    // relative to bodyA

        fx               := m_jibSail.GetFixtureList;
        edgeShp          := Tb2EdgeShape(fx.GetShape);     // we can typecast here cause we know how a rope is built..
        e := edgeShp.m_vertex2-edgeShp.m_vertex1;
        rjd.localAnchorB := MakeVector(0, -e.Length/2  );  // relative to sail seg //??
        rjd.maxLength    := 3;
        m_jibSheet       := Tb2RopeJoint( m_world.CreateJoint(rjd, true) );
      end;

      begin // Main Sail -------------------------------------
        for i:=0 to 5 do
          aVela[i].SetValue( VelaMain[i].x+m_mast.GetPosition.x ,
                             VelaMain[i].y+m_mast.GetPosition.y-0.31   ); //calc posição dos ptos da vela

        BuildRope( @aVela[0], 6, m_world, mainSailSegments, m_boatHull, nil, 0.1, maxseg );
        m_mainSail := Tb2Body(mainSailSegments.Last); //get vela

        // use distance joint to attach main sail to the boom
        jdst := Tb2DistanceJointDef.Create;
        jdst.Initialize(m_boom, m_mainSail,
           m_boom.GetPosition+MakeVector(0,-5.5)  ,       //anchorA absolute pos
           m_mainSail.GetWorldCenter+MakeVector(0,0)  );   //anchorB abs pos
        jdst.collideConnected :=false;
        m_mainOutHaul := Tb2DistanceJoint( m_world.CreateJoint(jdst, true) );

        // use a rope joint to attach the boom to the boat
        rjd       := Tb2RopeJointDef.Create;
        rjd.bodyA := m_boom;
        rjd.bodyB := m_boatHull;
        rjd.localAnchorA := MakeVector(0,-4.3  );  // relative to boom
        rjd.localAnchorB := MakeVector(0,-3.3  );   // relative to boat
        rjd.maxLength    := 3;
        m_mainSheet := Tb2RopeJoint( m_world.CreateJoint(rjd,true) );
      end;  // /main sail

      jibSailSegments.SetRopeAsSail;
      mainSailSegments.SetRopeAsSail;

      // jdst := Tb2DistanceJointDef.Create;
      // jdst.Initialize( m_island, m_boatHull, m_island[0].GetPosition, Proa  );
      // m_world.CreateJoint(jdst, true);

       // create distance joint to anchor the boat

      // use a rope if you want the boat swinging around
      // rjd := Tb2RopeJointDef.Create;    // tie boat to ground 0 by a rope
      // rjd.bodyA := m_island;
      // rjd.bodyB := m_boatHull;
      // rjd.localAnchorA := MakeVector(0,0);
      // rjd.localAnchorB := MakeVector(0,1);  //connect a little ahead of the mast
      // aVec := (m_boatHull.GetPosition-m_island.GetPosition);
      // rjd.maxLength    := aVec.Length;
      // rjd.collideConnected := true;
      // m_world.CreateJoint(rjd, true);


      // use a revolute jt if you want the boat to stay put
      // comentei pra soltar o bixo

      bd           := Tb2BodyDef.Create;  //create a ground for the boat
      bd.position  := boatInitPos;        //same placeas the boat, so the jt is invisible
      bd.bodyType  := b2_staticBody;
      m_groundBoat := m_world.CreateBody(bd, true);  //no fixture

      revj := Tb2RevoluteJointDef.Create;  //tie the boat to a point by a revj
      //revj.lowerAngle := 0;           // boom has a 90 deg range
      // revj.upperAngle := 0;
      revj.enableLimit:= false;
      revj.Initialize(m_boatHull, m_groundBoat, m_boatHull.GetPosition );
      revj.motorSpeed     := 0;        //result to unwanted rudder change
      revj.maxMotorTorque := 1000.0;
      revj.enableMotor    := true;

      revj.collideConnected := false;
      m_boatGroundJt := Tb2RevoluteJoint(m_world.CreateJoint(revj,true));

      // prevent created ropes from colliding w the boat hull (sails must be inside the boat sometimes)

      trueWind   := MakeVector( 0, 0);   //local copy of the true wind vector
      boatSpeed  := 0;
      boatCourse := 0;
      TWS:=0;   TWD:=0; TWA:=0;    //true w
      AWS:=0;   AWD:=0; AWA:=0;    //apparent w

      updateWindVectors(trueWind);
   end;   // /create sails

   m_AutoPilot := TAutoPilot.Create;
   //after creation, some sail trimming
   m_mainOutHaul.Length := 0.4;        //not too tight..
end;

destructor TSailboat.Destroy;
begin
  jibSailSegments.Free;
  mainSailSegments.Free;

  m_AutoPilot.Free;

  inherited;
end;

procedure TSailboat.ClearRopeSegments;  //and wind controller elements
var i: Integer;
begin
   for i := 0 to jibSailSegments.Count - 1  do m_world.DestroyBody(Tb2Body(jibSailSegments[i] ));
   jibSailSegments.Clear;

   for i := 0 to mainSailSegments.Count - 1 do m_world.DestroyBody(Tb2Body(mainSailSegments[i]));
   mainSailSegments.Clear;
end;

// measured from yaxis, positive clockwise
function TSailboat.getKeelAngleOfAttack:double;  //same as boat. In radians)
var BS,YAxis:TVector2;
begin
  BS      := m_boatHull.GetLinearVelocity;
  BS.Normalize;                        //needed ??
  YAxis   := m_boatHull.GetWorldVector( MakeVector(0,1) );
  Result  := _AngleBetweenVectors(BS,YAxis);
     _DrawText( Format('attack: %4.0f', [Result*180/Pi] ),  colorCINZA );  //teste
end;

function  TSailboat.getBoatSpeedVersor:TVector2;
begin
  Result := m_boatHull.GetTransform.q.GetYAxis;
  Result.Normalize;               //BS versor
end;

function  TSailboat.getBoatSpeedVec:TVector2;
begin
  Result := getBoatSpeedVersor;
  Result := Result*boatSpeed;
end;

Procedure TSailboat.updateWindVectors(const aTrueWind:TVector2);  // .. from true wind
begin //save a local copy of true wind
  trueWind     := aTrueWind;                       //  __   __   __
  apparentWind := trueWind - getBoatSpeedVec;     //   AW = TW - BS
end;

function TSailboat.getBoatForwardVelocity:TVector2;
var curYAxis:TVector2;
begin
  curYAxis := m_boatHull.GetWorldVector( MakeVector(0,1) );    // boat yaxis
  Result   := b2Dot( curYAxis, m_boatHull.GetLinearVelocity ) * curYAxis;
end;

function TSailboat.getBoatLateralVelocity:TVector2;
var curXAxis:TVector2;
begin
  curXAxis := m_boatHull.GetWorldVector( MakeVector(1,0) );    // boat xaxis
  Result   := b2Dot( curXAxis, m_boatHull.GetLinearVelocity ) * curXAxis;
end;

procedure TSailboat.ShowForceInSail(aSailSegments:TOmSail; timeStep: PhysicsFloat);
var p1,p2,F,Fu,Fd,a,b:TVector2; S:String;
begin
  if (aSailSegments.Count<2) then exit;
  // if bUseLDformula then _DrawText('L/D form') else _DrawText('regular form');

  p1 := aSailSegments.getSailCenter;
  F  := aSailSegments.SumF;          // get sum of L/D forces acting in the sail segments (updated by wind controller)
  p2 := p1+F*FORCE2SCREEN_SCALE;

  if bShowVectors then
    begin
       S := Format('%4.0f', [F.Length] ) ;
      _DrawVectorArrow(p1, p2, S, colorVerde);  //draw F vector
    end;

  // calc Lift and Drag components in relation to the boat movement
  // check: or should I have used boat y axis ?

  // project F in boat spd directions
  b := getBoatSpeedVec;
  b.Normalize;                     //  boat speed versor
  a  := MakeVector(-b.y,b.x );    // calc versor a, perpendicular to b. a points to the side of the boat
  Fu := b2Dot(F, b)*b;           // this projects the resulting force in the boat axis ( i.e. usefull force)
  p2 := p1+Fu*FORCE2SCREEN_SCALE;

  if bShowVectors then
    begin
       S := Format('%4.0f', [Fu.Length] ) ;
      _DrawVectorArrow(p1, p2, S, colorBlu);     // useful fwd force
    end;

  Fd := b2Dot(F, a)*a;                    // this force is sideways
  p2 := p1+Fd*FORCE2SCREEN_SCALE;
  if bShowVectors then
    begin
       S := Format('%4.0f', [Fd.Length] ) ;
      _DrawVectorArrow(p1, p2, S, colorRosso);     // sideways force
    end;

  // Cheat: Apply keel lift to balance sail sideways force
  //  c := m_boatHull.GetPosition;
  //  m_boatHull.ApplyForce( -Fd, c,{wakw=} True);

  //-----------------------------------------------------------------------------------
  // TODO: Since lift is required from the keel, so the boat can go forward,
  // it also means added drag (for lift is always attached to drag. How much ?
  //-----------------------------------------------------------------------------------
end;

procedure TSailboat.ApplyControlSettings(const jibSett,mainSett,rudderSet:Double);  //
var mSpd,jSpd,diff,adif,sj,sm,sr:Double;
const DEG2RAD=Pi/180;
begin
  // transform from range 0..1.0 to meters and rudder radians
  sj := MIN_JIB_SHEET_LEN +jibSett*(MAX_JIB_SHEET_LEN-MIN_JIB_SHEET_LEN);
  sm := MIN_MAIN_SHEET_LEN+mainSett*(MAX_MAIN_SHEET_LEN-MIN_MAIN_SHEET_LEN);
  sr := rudderSet*2*MINMAX_RUDDER_ANG-MINMAX_RUDDER_ANG;     //convert 0..1 rudder setting to angle in rads

  m_jibSheet.MaxLength   := sj;
  m_mainSheet.MaxLength  := sm;
  // use jt motor to turn the rudder
  mSpd := 0;   //=stopped
  diff := m_rudderCtrlJt.getJointAngle-sr;  //diff between actual rudder and skipper setting
  adif := Abs(diff);

  jSpd := m_rudderCtrlJt.GetJointSpeed;
  if (diff>0) then    //must currect tiller
    begin
      if      (adif< 3*DEG2RAD) then mSpd := 0.2
      else if (adif< 5*DEG2RAD) then mSpd := 0.3
      else if (adif<10*DEG2RAD) then mSpd := 0.7
      else if (adif<15*DEG2RAD) then mSpd := 1.0
      else mSpd := 1.2;   //max speed
      mSpd := -mSpd;      //negative
    end
    else if (diff<0) then
    begin
      if      (adif< 5*DEG2RAD) then mSpd := 0.2
      else if (adif<10*DEG2RAD) then mSpd := 0.3
      else if (adif<15*DEG2RAD) then mSpd := 0.7
      else if (adif<20*DEG2RAD) then mSpd := 1.0
      else mSpd := 1.2;   //max speed
    end;
  m_rudderCtrlJt.SetMotorSpeed(mSpd);
end;

procedure TSailboat.ShowBoatCourseSpeed; //and speed
var cap,rudderSetting:Double;  rTWD:Double;
begin
  cap        := -m_boatHull.GetTransform.q.GetAngle;
  if (cap<0) then cap:=cap+2*Pi;                         //0..2 Pi range
  boatCourse := cap*180/Pi;
  _DrawText(Format('Boat COG: %4.0f° Spd: %5.2f Kn',[ boatCourse, boatSpeed*MpS2Kn ]), colorVerde );

  // trecho extraido de Trigonom.pas
  // calc True Wind Angle TWA

  rTWD := TWD+180;                   // change from "wind-to" to "wind-from"
  if rTWD>360 then rTWD:=rTWD-360;   // 0..360

  if (rTWD>boatCourse) then TWA:=rTWD-boatCourse
    else TWA:=boatCourse-rTWD;        //basicaly TWA:=Abs(course-TWD)             TWA=true wind angle
  if (TWA>180) then TWA:=TWA-180     //put TWA in the 0..180 range
     else TWA:=180-TWA;
  // apply auto pilot TODO: move to its own proc
  _DrawText(Format('TWA: %4.0f°',[TWA]) ,colorVerde );

  if m_AutoPilot.Enabled then
     begin
       rudderSetting := m_AutoPilot.getRudderSetting( boatCourse,boatSpeed );  //range 0..1   (0.5=even)
         _DrawText( Format('Auto pilot: %3.0f  %4.0f°', [ (rudderSetting-0.5)*100, m_AutoPilot.TargetCourse ]), colorVerde );
       if Assigned(m_OnAutoPilotAction) then m_OnAutoPilotAction(rudderSetting);
     end;
end;

function TSailboat.getBoatPosition:TVector2;
begin
  Result := m_boatHull.GetPosition;
end;

procedure TSailboat.ApplyForcesToBoat;    // rudder forces, keel forces and some cheats
const MAX_RUDDER_FORCE=2000;   //
      RoWater=10.0;           // ?? water density  (heavy)
      RudderFoilLen=0.25;    //rudder 10% of keel
      KeelFoilLen=2.5;

var F,bs,bsv,b,p1,p2,impulse,v1,v2,posF:TVector2;  //vectors
    xf:Tb2Transform;
    a:double;
    S:String;

begin
  // Sail forces are applied by wind controller to individual sail segments.
  bs := m_boatHull.GetLinearVelocity;
  boatSpeed := bs.Length;     // update boatspeed scalar with m_boatHull linear speed
  b := - getBoatSpeedVersor;  // set b = versor pointing back

  begin    // show keel attack angle (i.e. boat linear speed)
    p1 := m_boatHull.GetPosition;
    p2 := p1+bs*10;

    S := Format('%4.1f', [boatSpeed*MpS2Kn] );
    _DrawVectorArrow(p1, p2, S, colorCINZA );     //draw boat speed vector

    // keel angle is the angle betweem boat y-axis and boat movement.
    // Should be near 0 most of the time (water density is high and keel lift grows fast with alpha)
    a := Abs( _AngleBetweenVectors(bs,b)*180/Pi );
    a := 180-a;  // b points back, so get complementary angle
    _DrawText( Format('keel angle: %4.1f°',[a]), colorCINZA );
  end;

  // Apply Keel L/D force
  v1 := MakeVector(0, -1);         // make keel  a 2.5 m long segment v1-v2
  v2 := MakeVector(0,1.5);
  xf := m_boatHull.GetTransform^;  // same xf as the hull
  CalcLiftDragWaterfoil(v1,v2,bs,xf,F);          // calc F = lift+drag
  posF := m_boatHull.GetPosition-b/2;           // position of the keel center, a little fwd of boat center
  m_boatHull.ApplyForce( F, posF,{wake=}true); // <------  Apply keel force
  if bShowVectors then
    begin
      p1 := posF;   p2 := p1+F*FORCE2SCREEN_SCALE;
      S := Format('%4.0f', [F.Length] ) ;
     _DrawVectorArrow(p1, p2, S, colorWine);     // Keel F
    end;

  // Apply rudder L/D force to rudder
  v1 := MakeVector(0, 0);       // rudder segment v1-v2
  v2 := MakeVector(0, 0.3);
  xf := m_rudder.GetTransform^;
  CalcLiftDragWaterfoil(v1,v2,bs,xf,F);
  // take care not to create too much rudder torque
  // use small dist from rudder jt to force
  posF := m_rudder.GetPosition+b/7;           // pos of rudder force, in the rudder blade
  m_rudder.ApplyForce( F, posF,{wake=}true); // <------  Apply rudder force

  if bShowVectors then
    begin
      p1 := posF;   p2 := p1+F*FORCE2SCREEN_SCALE;
      S := Format('%4.0f', [F.Length] ) ;
     _DrawVectorArrow(p1, p2, S, colorWine);
    end;

  // Apply hull form/friction force, proportional to BS^2. Proportional to boats weight
  // this plus the keel drag are the forces

  bsv := bs;      // use boatspeed vector
  bsv.Normalize;  // mk a versor

  // both surface drag and boat form drag are prop to sq velocity.

  F := - (12*boatSpeed*boatSpeed*bsv);      // k*boat weight=6

  posF := m_boatHull.GetPosition;
  m_boatHull.ApplyForce( F, posF,{wake=}true); // <------------  Apply hull form drag + surface drag

  if bShowVectors then
    begin
      p1 := posF;   p2 := p1+F*FORCE2SCREEN_SCALE;
      S := Format('%4.0f', [F.Length] ) ;
     _DrawVectorArrow( p1, p2 ,S ,colorHDrag );         //show hull drag
    end;

  //if false then  //disable cheats
  begin    // Cheats to achieve more realistic simulation...  :(  keep this to a minimum !
    //  Kill sideways movement. Uses idea from a car game tutorial,
    //  to kill tire sideways movement by applying impulses and torque
    //  see http://www.iforce2d.net/b2dtut/top-down-car

    // cheat #1
    // impulse := m_boatHull.GetMass* -getBoatLateralVelocity();
    // m_boatHull.ApplyLinearImpulse( impulse*0.05, m_boatHull.GetWorldCenter );  // kill some sideways movement

    // cheat #2
    m_boatHull.ApplyAngularImpulse( 0.05 * m_boatHull.GetInertia* -m_boatHull.GetAngularVelocity ); //also kill some rotation

    // cheat #3: Avoid backward movement ?
    // bsy := getBoatLateralVelocity;
    // r   := b2Dot(bs,b);
    // if (r<0) then // pra traz nao. kill backward movement
    //    begin
    //     // _DrawText('v neg!');
    //     impulse := m_boatHull.GetMass* -bsy ;
    //     m_boatHull.ApplyLinearImpulse( impulse, m_boatHull.GetWorldCenter );
    //    end;
  end;
end;

procedure TSailboat.UpdateInstruments(timeStep: PhysicsFloat);
var aChord,aCamber,alfa:Double;
begin
   AWS := apparentWind.Length*MpS2Kn;
   AWD := _AngleBetweenVectors( ApparentWind, MakeVector(1,0) )*180/Pi-90;
   if AWD<0 then AWD:=AWD+360;


   _DrawText(Format('App  W: %4.1f kn  %4.0f°', [AWS,AWD] ), colorVerde );

   // DrawText(Format('True W: (X: %f, Y: %f) %4.1f Kn',    [trueWind.x, trueWind.y,TrueWind.Length*MpS2Kn] ));
   TWS := TrueWind.Length*MpS2Kn;
   TWD := _AngleBetweenVectors( TrueWind, MakeVector(1,0) )*180/Pi-90;
   if (TWD<0) then TWD:=TWD+360;

   _DrawText( Format('True W: %4.1f Kn  %4.0f°',[TWS,TWD]), colorVerde  );

   _DrawText(Format('jib: %4.1f outhaul: %4.1f main: %4.1f', [m_jibSheet.maxLength,m_mainOutHaul.Length,m_mainSheet.MaxLength] ), colorVerde );

   ShowForceInSail( jibSailSegments,  timeStep ); // show force and other sail properties. Also applies keel force to counterbalance
   ShowForceInSail( mainSailSegments, timeStep );

   ShowBoatCourseSpeed;

   getSailChordCamber('jib',jibSailSegments,  aChord,aCamber,alfa);    //show sail chord and draft
   getSailChordCamber('main',mainSailSegments,aChord,aCamber,alfa);
end;

Procedure TSailboat.getSailChordCamber( const aSailname:String; aList:TList; var aChord,aCamber,alfa:Double );  //aList is a TList of Tb2Body s
var aRopeSeg1,aRopeSeg2: Tb2Body;
    fx :Tb2Fixture;
    p1,p2,p3,aChordV,aV,aP,aAW:TVector2;
    d,dmax,aApW:double;
    i:integer;
    aEdge1,aEdge2:Tb2EdgeShape;
    xf: Pb2Transform;
    S:String;
begin
  if (aList.Count>2) then //sanity test
    begin
      aRopeSeg1 := Tb2Body(aList[0]);               //1st segment (front)
      aRopeSeg2 := Tb2Body(aList[aList.Count-1]);  //last segment (sheet)

      // draw sail chord, from pt 1 of 1st segment to last pt of last segment
      fx     := aRopeSeg1.GetFixtureList;
      aEdge1 := Tb2EdgeShape(fx.GetShape);      //we can typecast here cause we know how a rope is built..
      aP     := aRopeSeg1.GetPosition;
      xf     := aRopeSeg1.GetTransform;


      p1     := b2Mul(xf^, aEdge1.m_vertex1);
      p2     := b2Mul(xf^, aEdge1.m_vertex2);
      aV     := (p2-p1);     // chord vector
      if bShowVectors then
        begin
          S := ''; //no txt
          _DrawVectorArrow( p1, p2 ,S , colorVerde);     //draw sail chord (blue)
        end;

      aV.Normalize;

      aAW  := apparentWind;                             // draw apparent wind vector
      aApW := apparentWind.Length*SPEED2SCREEN_SCALE;   // scale factor for speed vec
      if bShowVectors then
        begin
          S := Format('%4.0f', [apparentWind.Length*MpS2Kn] ) ;
         _DrawVectorArrow(p1,p1+aAW, S, colorAppW);   //apparent wind vector is brown
        end;

      aAW.Normalize;  //normalize for alfa calc

      alfa := ArcCos( b2Dot(aV,aAW))*180/Pi;

      fx     := aRopeSeg2.GetFixtureList;
      aEdge2 := Tb2EdgeShape(fx.GetShape);
      aP     := aRopeSeg2.GetPosition;
      p2     := aRopeSeg2.GetPosition+aEdge2.m_vertex2;

      aChordV := p2-p1;

      if bShowVectors then
        begin
           S := ''; //no txt
          _DrawVectorArrow(p1, p2,S, colorBlu);     //draw chord
        end;

      dmax:=0;
      for i:=1 to aList.Count-2 do
        begin
          aRopeSeg1 := Tb2Body(aList[i]);              //1st
          p3 := aRopeSeg1.GetWorldCenter;
          d:= PointToSegmentDistance(p1,p2,p3);
          if d>dmax then dmax := d;  //save
        end;

      aChord  := aChordV.Length;
      aCamber := dmax;

      // _DrawText( Format(aSailname+' chord: %4.1f draft: %4.2f alfa: %3.0f AW: %4.1f', [aChord,aCamber,alfa,aApW]) );  //show force  aMA_force
    end;
end;

procedure TSailboat.Step(var settings: TSettings; timeStep: PhysicsFloat);
begin
  // note: here trueWind and apparentWind must be updated already
  ApplyForcesToBoat;            // forces: keel (lift/drag) + rudder(lift/drag) + hull drag
  UpdateInstruments(timeStep);
end;

procedure TSailboat._DrawText(const text: string; const color: RGBA);
begin
  if Assigned(m_OnDrawText) then m_OnDrawText(text,color);
end;

Procedure  TSailboat._DrawVectorArrow(const p1, p2: TVector2; const Text:String; const color: RGBA);
begin
  if Assigned(m_OnDrawVector) then m_OnDrawVector(p1,p2,Text, color);
end;

//--------------------------------------------------------
Procedure ScaleBoatSeta;
var i:integer;
begin
  for i := 0 to 9 do //scale boat 10x
    begin LateralBarco[i].x := LateralBarco[i].x*10; LateralBarco[i].y := LateralBarco[i].y*10; end;
  for i := 0 to 4 do
    begin
      VelaJib[i].x := VelaJib[i].x*10; VelaJib[i].y := VelaJib[i].y*10;
    end;
  for i := 0 to 5 do
    begin
      VelaMain[i].x := VelaMain[i].x*10; VelaMain[i].y := VelaMain[i].y*10;
    end;
end;

initialization
   ScaleBoatSeta;

end.
