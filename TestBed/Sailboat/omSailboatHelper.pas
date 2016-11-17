Unit omSailboatHelper;  // classes used by sailboat
 //--------------------//
//   by Omar. out16
//   a sail boat physics model for box2d
//      TExponentialMovingAverageVector - calculator of EM average of a vector
//      TOmSail - A bunch of egdeshape bodies, jointed by revolute jts
//      TSailboatWindController - similar to Tb2WindController, with lift/drag
//      TAutoPilot - automatic steering
//      TBoatWake - particles container for boat wake bubbles


interface
{$I ..\..\Physics2D\Physics2D.inc}

uses
   System.Classes,    // TList
   System.SysUtils,   // Now
   System.Math,
   // b2d
   UPhysics2DTypes, UPhysics2D, UPhysics2DControllers;

var   //global vars
  bUseLDformula:boolean=true;   // (recommended) use L/D formula instead of b2d original drag-only version, for test
  maxseg:Single=0.65;           // max segment size, used to create ropes and sails
  bShowVectors:boolean=false;   // show vectors for boatspeed, forces in sails,keel&rudder

const
   // selective collision masks
   CAT_LAND=$0001;   // island
   CAT_BOAT=$0002;   // boat collides with other boats, land. Not to sails
   CAT_SAIL=$0004;   // sails cannot collide with boat, but do collide with mast and other sails..
   CAT_MAST=$0008;   // mast colides with sails, but wants to be inside the boat
   CAT_BOOM=$0010;   // boom cat doesnt collide with nothing
   CAT_BUBBLE=$0020; // sea waves and boat wake bubbles
   // hit masks
   MASK_ALL =$ffff;
   MASK_NONE=0;
   MASK_NO_SAIL=MASK_ALL xor CAT_SAIL;
   MASK_NO_BOAT=MASK_ALL xor CAT_BOAT;
   MASK_NO_SAIL_MAST_BOOM=MASK_ALL xor CAT_SAIL xor CAT_MAST xor CAT_BOOM;

   MpS2Kn=3600/1852;   //  speed conversion ( m/s --> knots )
   MAX_BUBBLES=300;    // max number of bubbles floating

Type
   // Controls wake bubbles life span
   TBubbleCreationRec=class
   private
      m_expirationTime:TDatetime;
   public
      Constructor Create;
      function Expired:boolean;  // expired bubbles are terminated
   end;

   // Exponential moving average of a vector, to smooth an aggitated variable
   TExponentialMovingAverageVector=class
   private
     prevValue:TVector2;
     curValue:TVector2;
     m_nPer:integer;
     m_K:double;             //exponential const
   public
      Constructor Create(NumPer:integer);          // NumPer=number of periods
      function    AddPt(const aValue:TVector2):TVector2;
      function    getValue:TVector2;
   end;

   // TOmSail - A bunch of egde shaped bodies, jointed by revolute joints, as build by BuildRope()
   TOmSail=class(TList)   // TList of Tb2Body s
   private
   public
     SumF:TVector2;   //sum of the forces in all segments of the sail
     sailName:String;
     forceAvg:TExponentialMovingAverageVector;     // average sail force

     Constructor Create(const aSailname:String);
     Destructor  Destroy; override;
     function    getSegment(i:integer):Tb2Body;
     Procedure   SetRopeAsSail;           //set sail physical properties, not set by BuildRope()
     function    getSailCenter:TVector2;
   end;

   // TSailboatWindController - similar to Tb2WindController, with more natural lift/drag model
   TSailboatWindController = class(Tb2Controller)
   private
      function ComputePolygonShapeEffectiveForce(const windforce: TVector2; shape: Tb2PolygonShape; const xf: Tb2Transform): TVector2;
      function ComputeEdgeShapeEffectiveForce(const windforce: TVector2; shape: Tb2EdgeShape; const xf: Tb2Transform): TVector2;
   public
      Callback: Tb2WindForceCallback;
      Sails:TList;

      Constructor Create;
      Destructor  Destroy; override;
      procedure   ClearSails;
      procedure   AddSail(aSail:TOmSail);
      procedure   Step(const step: Tb2TimeStep); override;
   end;

  // TAutoPilot - tentative auto pilot. TODO: Oscillates too much.
  TAutoPilot=class                        //  Needs improved algo..
  private                                // Looks like a drunk captain..
     m_Enabled:boolean;
     m_lastChg:TDatetime;
     m_rudderCfg:Double;
     m_rudderBias:Double;    //auto trim
     m_rudderGain:Double;    //gain
     m_targetCourse:Double;
  public
     constructor Create;
     function    getRudderSetting(const boatCourse,boatSpeed:Double):double; // return auto pilot suggested rudder setting ( 0..1.0 )
     procedure   setCourse(const boatCourse:Double);    //set target course
     Property    TargetCourse:Double  read m_targetCourse;
     // TODO: auto TWA function
     Property Enabled:boolean      read m_Enabled write m_Enabled;             // =engaged
  end;

  TBoatWake=class
  private
  public
    lastBubbleCreated:TDatetime; // wake bubbles making cadence
    m_particles:TList;    // boat wake particles
    Constructor Create;
    Destructor  Destroy; override;
  end;

// Om: exposed fn for test
procedure _ComputeSegmentEffectiveForceLD(const first, second: TVector2;
   normal: TVector2; const xf: Tb2Transform; const windforce: TVector2; const RoFluid:Double; var accumulated: TVector2);

//          P2      PointToSegmentDistance:
//          /       Line P1,P2 (  parametric equ: P = P1 + u(P2-P1)   --> for the segmt  0 <= u <= 1.0
//         /        PT é o pto de P1-P2 mais proximo de P3
//     PT /
//       + _  D
//      /    - _
//     /         +P3
//   P1
function  PointToSegmentDistance(const P1,P2,P3:TVector2):double;
function  Pt2Pt_Distance(const P1,P2:TVector2):double;

implementation  //-------------------------------------------------------------------------------------

uses
   uLiftDragCalculator;     //  calc lift / drag coefficients for flat plates

procedure _ComputeSegmentEffectiveForce(const first, second: TVector2;
   normal: TVector2; const xf: Tb2Transform; const windforce: TVector2;
   var accumulated: TVector2);
var
   edge: TVector2;
begin
   normal := b2Mul(xf.q, normal); // to world coordinate
   {$IFDEF OP_OVERLOAD}
   edge := b2Mul(xf.q, second - first);
   {$ELSE}
   edge := b2Mul(xf.q, Subtract(second, first));
   {$ENDIF}
   if b2Dot(windforce, normal) < 0 then // this edge faces the wind
   begin
      if windforce.x > 0 then accumulated.x := accumulated.x + Abs(windforce.x * edge.y)
                         else accumulated.x := accumulated.x - Abs(windforce.x * edge.y);

      if windforce.y > 0 then accumulated.y := accumulated.y + Abs(windforce.y * edge.x)
                         else accumulated.y := accumulated.y - Abs(windforce.y * edge.x);
   end;
end;

// Om: Out16: this is derived from _ComputeSegmentEffectiveForce()
// Instead of just doing accumulated.x := accumulated.x + Abs(windforce.x * edge.y)
// which basically says that F = drag, the added lift means boat sails inflate more naturally
procedure _ComputeSegmentEffectiveForceLD(const first, second: TVector2;
   normal: TVector2; const xf: Tb2Transform; const windforce: TVector2;
   const RoFluid:Double; var accumulated: TVector2 );
var
   edge,aV,aW,aForce:TVector2;
   alpha,alphadeg,CL,CD,L,FD,FL,WS,K,sn,cs,wang:Double;
   sig:integer;
begin
   normal := b2Mul(xf.q, normal); // to world coordinate
   {$IFDEF OP_OVERLOAD}
   edge := b2Mul(xf.q, second - first);     // edge vector
   {$ELSE}
   edge := b2Mul(xf.q, Subtract(second, first));
   {$ENDIF}
   if (b2Dot(windforce, normal)<0) then   // negative dot prod here means this edge faces the wind
   begin
      aV := edge;          // = airfoil chord
      aV.Normalize;
      aW  := windforce;    //draw apparent wind vector (wine)
      aW.Normalize;       //normalize for alpha calc

      alpha := ArcCos( b2Dot(aV,aW) );  // calc angle of attack
      //alpha := Abs(alpha);            // negative angle of attack ?? not here..

      sig:=+1;
      alphadeg := alpha*180/Pi;          // sig is the lift direction (up/down)
      if    (alphadeg>90.0) then begin alphadeg:=180-alphadeg; sig:=-1;  end
      else if (alphadeg<0)  then begin alphadeg:=-alphadeg;    sig:=-1;  end;

      // yeah.. not really a paddle here. But paddle coefficients are ok for simple simulation
      // ideally one should use some more complex aerodynamic model
      if Angle2LiftDragForPaddle( alphadeg, CL, CD ) then  // get lift / drag coeffs
        begin
          // CD,CL are in relation to wind vector
          WS := windforce.Length;
          //  Lift/ Drag forces in 2D can be calculated by
          //  FD := 1/2 * Ro * W^2 * L * CD    FL := 1/2 * Ro * W^2 * L * CL
          //  where: FD=drag force FL=lift force Ro=fluid density  W=wind spd L=airfoil chord  CL=lift coeff CD=drag coeff
          //  coefficients are found by airfoil tests (table based)
          K  := 1/2*RoFluid*WS*WS*edge.Length;  // memoise K = 1/2 * Ro * W^2 * L
          FD := K*CD;
          FL := Sig*K*CL;
          // FD,FL are in relation to wind direction, so rotate FD,FL to absolute

          // rotate F with wind angle to get a real world vector
          wang := ArcTan2( aW.y, aW.x);   //calc wind angle
          cs := cos(wang);  sn := sin(wang);
          aForce.x := FD * cs - FL * sn;    // add FD, FL and rotate by wang
          aForce.y := FD * sn + FL * cs;

          accumulated.x := accumulated.x + aForce.x;   //add by
          accumulated.y := accumulated.y + aForce.y;
        end;
   end;
end;

function Pt2Pt_Distance(const P1,P2:TVector2):double;
var dx,dy:double;
begin
  dx:=P2.x-P1.x; dy:=P2.y-P1.y;
  Result:=Sqrt(dx*dx+dy*dy);
end;

function PointToSegmentDistance(const P1,P2,P3:TVector2):double;
var u,dx,dy,d:double; PT:TVector2;
begin
  u := (P3.x-P1.x)*(P2.x-P1.x)+(P3.y-P1.y)*(P2.y-P1.y);
  dx:= P2.x-P1.x; dy:=P2.y-P1.y;
  d := (dx*dx+dy*dy);
  if d=0 then PT:=P1   //P1=P2 --> line P1-P2 is a single point --> PT=P1
    else begin
      u:=u/d;
      if (u<=0) then PT:=P1
      else if (u>=1.0) then PT:=P2
      else begin          //PT between P1 and P2, calc position
        PT.x := P1.x+u*(P2.x-P1.x);
        PT.y := P1.y+u*(P2.y-P1.y);
      end;
    end;
  Result:=Pt2Pt_Distance(P3,PT);
end;

{ TSailboatWindController }

Constructor TSailboatWindController.Create;
begin
  inherited;
  Sails := TList.Create;
end;

Destructor  TSailboatWindController.Destroy;
begin
  Sails.Free;
  inherited;
end;

procedure TSailboatWindController.ClearSails;
begin
  Sails.Clear;
end;

procedure TSailboatWindController.AddSail(aSail:TOmSail);
begin
  Sails.Add(aSail);
end;

procedure TSailboatWindController.Step(const step: Tb2TimeStep);
const DefaultWindForce: TVector2 = (X: 10; Y: 0); // From left to right
var
   aBody:Tb2Body;
   aShape:Tb2Shape;
   windforce: TVector2;
   appliedforce: TVector2;
   i,j:integer;
   edgeShp:Tb2EdgeShape;
   aSail:TOmSail;

begin
   if Assigned(Callback) then windforce := Callback()      // get wind vector
   else windforce := DefaultWindForce;

   for i:=0 to Sails.Count-1 do
     begin
       aSail := TOmSail(Sails.Items[i]);
       aSail.SumF := b2Vec2_Zero;       // sumF = soma das forças em ema vela
       for j := 0 to aSail.Count-1 do
         begin
           appliedforce := b2Vec2_Zero;   // appliedforce = soma das forças em um segmento
           aBody  := aSail.getSegment(j);
           aShape := Tb2Shape( aBody.GetFixtureList.GetShape );  //this has to be a Tb2EdgeShape, as built by BuildRope()
           if  (aShape is Tb2EdgeShape) then //must be !
             begin
                edgeShp := Tb2EdgeShape(aShape);
                {$IFDEF OP_OVERLOAD}
                appliedforce.AddBy( ComputeEdgeShapeEffectiveForce(windforce, edgeShp, aBody.GetTransform^) );
                {$ELSE}
                appliedforce.AddBy(appliedforce, ComputeEdgeShapeEffectiveForce(windforce, edgeShp, aBody.GetTransform^));
                {$ENDIF}
                aSail.SumF := aSail.SumF + appliedforce;                     // memoise total force in the sail
                aBody.ApplyForce( appliedforce, aBody.GetWorldCenter, True); // apply wind force to segment
             end;
         end;
       aSail.forceAvg.AddPt(aSail.sumF);  //calc average force
     end;
end;

function TSailboatWindController.ComputePolygonShapeEffectiveForce(const windforce: TVector2;
   shape: Tb2PolygonShape; const xf: Tb2Transform): TVector2;
var
   i: Integer;
begin
   Result := b2Vec2_Zero;
   with shape do
   begin
      if m_count < 2 then
         Exit;
      for i := 1 to m_count - 1 do
         _ComputeSegmentEffectiveForce(m_vertices[i - 1], m_vertices[i],
            m_normals[i - 1], xf, windforce, Result);

      _ComputeSegmentEffectiveForce(m_vertices[m_count - 1],
         m_vertices[0], m_normals[m_count - 1], xf, windforce, Result);
   end;
end;

//since this is only used by the wind controller, use air as fluid
function TSailboatWindController.ComputeEdgeShapeEffectiveForce( const windforce: TVector2; shape: Tb2EdgeShape; const xf: Tb2Transform): TVector2;
const RoAir=1.2;    // air density in Kg/m3
begin
   Result := b2Vec2_Zero;
   with shape do
   begin
     if bUseLDformula then    // Omar: added a more realistic lift/drag model LD
       begin
         _ComputeSegmentEffectiveForceLD(m_vertex1, m_vertex2, m_normal1, xf, windforce, RoAir, Result);
         _ComputeSegmentEffectiveForceLD(m_vertex2, m_vertex1, m_normal2, xf, windforce, RoAir, Result);
       end
       else begin
         _ComputeSegmentEffectiveForce(m_vertex1, m_vertex2, m_normal1, xf, windforce, Result);  // box2d default behaviour
         _ComputeSegmentEffectiveForce(m_vertex2, m_vertex1, m_normal2, xf, windforce, Result);
       end;
   end;
end;

{ TExponentialMovingAverageVector }
Constructor TExponentialMovingAverageVector.Create(NumPer:integer);       //number of periods
begin
  inherited Create;
  prevValue := MakeVector(0,0);
  curValue  := MakeVector(0,0);

  m_nPer := NumPer;
  m_K    := 2/(NumPer+1);     //exponential const
end;

function TExponentialMovingAverageVector.AddPt(const aValue:TVector2):TVector2;
begin
   if (prevValue.x=0) and (prevValue.y=0) then prevValue := aValue;   //init prev if zero
   curValue := (aValue-prevValue)*m_K + prevValue;                   //calc EMA
   Result := curValue;
end;

function TExponentialMovingAverageVector.getValue:TVector2;
begin
  Result := curValue;
end;

{ TOmSail }
Constructor TOmSail.Create(const aSailname:String);
begin
  inherited Create;
  sailName := aSailname;
  forceAvg := TExponentialMovingAverageVector.Create({NumPer=}60 ); // average sail force ( 60 = 1 sec +-)
end;

Destructor TOmSail.Destroy;
begin
  forceAvg.Free;
  inherited;
end;

function  TOmSail.getSegment(i:integer):Tb2Body;
var aBody:Tb2Body;
begin
  Result := nil;
  if (i>=0) and (i<Count) then
     Result := Tb2Body(Items[i]);
end;

function TOmSail.getSailCenter:TVector2;
var i:integer;  aBody:Tb2Body;
begin
  if (Count=0) then
        begin
          Result := MakeVector(0,0);  //??
          exit;
        end;

  i := Count div 3; //??
  aBody := Tb2Body(Items[i]);
  Result := aBody.GetPosition;
end;

Procedure TOmSail.SetRopeAsSail;  // TList of Tb2Body s connected by revolute joints
var i:integer; aRopeSeg: Tb2Body;
    fx :Tb2Fixture;
    filter:Pb2Filter;
    aJtEdge:Pb2JointEdge;
    aJt:Tb2Joint;
    aMD:Tb2MassData;
begin
  for i:=0 to Count - 1 do    // all edges
    begin
      aRopeSeg  := Tb2Body(Items[i]);
      aRopeSeg.GetMassData(aMD);
      aMD.mass  := 2.6;
      aMD.i     := 2.8;
      //aMD.center:= MakeVector(0,0);  //?
      aRopeSeg.SetMassData(aMD);

      fx := aRopeSeg.GetFixtureList;
      while Assigned(fx) do
        begin
          filter := fx.GetFilterData;
          filter^.categoryBits := CAT_SAIL;        // I'm a sail. I can be inside the boat
          filter^.maskBits     := MASK_NO_BOAT;   // sail will not colide with boat, so it can be inside
          filter^.groupIndex   := 0;             // 0 means use category
          fx.SetFilterData(filter^);
          fx := fx.GetNext;
        end;

     aJtEdge := aRopeSeg.GetJointList;
     aJt := aJtEdge^.joint;
     if (aJt is Tb2RevoluteJoint) then
       begin
          Tb2RevoluteJoint(aJt).EnableLimit(false);
          Tb2RevoluteJoint(aJt).SetMotorSpeed(0);
          Tb2RevoluteJoint(aJt).SetMaxMotorTorque( 10.0 );
          Tb2RevoluteJoint(aJt).EnableMotor(true);
       end;
   end;
end;

{ TBubbleCreationRec }
Constructor TBubbleCreationRec.Create;
const MIN_LIFE=3.0/3600/24; MAX_LIFE=8.0/3600/24; //  bubble life span random between 3 and 8 seconds
begin
  inherited;
  m_expirationTime := Now + RandomFloat( MIN_LIFE, MAX_LIFE );  // Set ramdom life span on creation
end;

function TBubbleCreationRec.Expired:boolean;
begin
  Result := (Now>m_expirationTime);  // oh oh
end;

{ TAutoPilot }
constructor TAutoPilot.Create;
begin
  inherited;
  m_Enabled       :=false;
  m_targetCourse  :=0;
  m_lastChg       :=0;      //=never
  m_rudderCfg     :=0.50;  //=centered rudder
  m_rudderBias    :=0;    //auto trim
  m_rudderGain    :=1.0;
end;

// return current rudder motor setting when the auto pilot is on
// this implements AP logic to correct the boat course
function TAutoPilot.getRudderSetting(const boatCourse,boatSpeed:Double):Double; //boatCourse in degrees
const xTEMPO=1/20/3600/24;
var t:TDatetime;  d,targetRudderCfg,incr,delta,deltaRud:Double; sig:integer;
begin
  if m_enabled then
    begin
      t := Now;
      //only take actions one time per second, to avoid overcontrolling
      if (t-m_lastChg>xTEMPO) then
        begin
          // Calc difference between curren COG and target
          d := m_targetCourse-boatCourse;                // target-boatcourse.   d negative means "turn right to go to target"
          if (d>180)       then begin d:=360-d; end     // better turn to the other side
          else if (d<-180) then begin d:=360+d; end;   //
          // table based logic
          sig:=+1;  //correction sinal
          if (d<0) then begin sig:=-1;  d:=-d; end;  // d=Abs(d)
          if (d<>0) then
            begin
              // calc gain - see  http://www.cactusnav.com/newsdesk_info.php?newsPath=11&newsdesk_id=10
              // as the speed grows, smaller rudder angles should be applied, so use decreasing gains
              if      (boatSpeed<3) then  m_rudderGain := 1.0     //large speeds, lo gain
              else if (boatSpeed<6) then  m_rudderGain := 0.8     //large speeds, lo gain
              else if (boatSpeed<10) then m_rudderGain := 0.6
              else if (boatSpeed<15) then m_rudderGain := 0.2
              else                        m_rudderGain := 0.1;    //small gain if going fast

              delta :=0;                 // delta=departure from centered rudder
              // auto pilot logic. very primitive.
              if      (d<=2) then begin  delta := sig*0.01; end   // sml diffs, sml chgs
              else if (d<=5) then begin  delta := sig*0.05; end
              else if (d<10) then begin  delta := sig*0.10; end
              else                begin  delta := sig*0.20; end;   // keep in -0.7..0.7
              targetRudderCfg := 0.50+delta*m_rudderGain;         // 0.50 = centered rudder. apply gain
              // have a target. Decided what to do with the rudder
              incr := targetRudderCfg-m_rudderCfg;
              // theres no point turning rudder too hard since it stols after 40 deg. Keep in 30-70%

              deltaRud := 0.05;     // change allowed in m_rudderCfg each tick
              if      (incr>0) and (m_rudderCfg<0.7) then m_rudderCfg := m_rudderCfg+deltaRud   // left or right ?
              else if (incr<0) and (m_rudderCfg>0.3) then m_rudderCfg := m_rudderCfg-deltaRud;  // slowly go to rudder target
            end;
          m_lastChg := t;   // save time
        end;
    end;
  Result := m_rudderCfg;  //return suggested rudder setting ( 0..1.0 range     0.5 means centered )
end;

procedure   TAutoPilot.setCourse(const boatCourse:Double);
begin
  m_targetCourse := boatCourse;
  m_lastChg      := now;
  m_rudderCfg    := 0.50;    //=centered
end;

{ TBoatWake }

constructor TBoatWake.Create;
begin
  inherited Create;
  m_particles := TList.Create;
  lastBubbleCreated:=0; //=never
end;

destructor TBoatWake.Destroy;
begin
  m_particles.Free;
  inherited;
end;

end.

