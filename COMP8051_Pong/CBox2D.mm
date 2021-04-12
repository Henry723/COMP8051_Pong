//
//  Copyright © Borna Noureddin. All rights reserved.
//

#include <Box2D/Box2D.h>
#include "CBox2D.h"
#include <stdio.h>
#include <map>

// Some Box2D engine paremeters
const float MAX_TIMESTEP = REFRESH_RATE;
const int NUM_VEL_ITERATIONS = 10;
const int NUM_POS_ITERATIONS = 3;

class UserData{
public:
    CBox2D* box2D;
    NSString* objectName;
    UserData(CBox2D* box, NSString* name){
        box2D = box;
        objectName = name;
    }
};

#pragma mark - Box2D contact listener class

// This C++ class is used to handle collisions
class CContactListener : public b2ContactListener
{
public:
    void BeginContact(b2Contact* contact) {};
    void EndContact(b2Contact* contact) {};
    void PreSolve(b2Contact* contact, const b2Manifold* oldManifold)
    {
        b2WorldManifold worldManifold;
        contact->GetWorldManifold(&worldManifold);
        b2PointState state1[2], state2[2];
        b2GetPointStates(state1, state2, oldManifold, contact->GetManifold());
        if (state2[0] == b2_addState)
        {
            // Use contact->GetFixtureA()->GetBody() to get the body
            b2Body* bodyA = contact->GetFixtureA()->GetBody();
            
            //first check if the bodys userdata is not null
            if(bodyA->GetUserData() != NULL){
                
                //body data currently contains
                UserData* bodyData = (UserData*)(bodyA->GetUserData());
                CBox2D *parentObj = bodyData->box2D;
                // Call RegisterHit (assume CBox2D object is in user data)
                if([bodyData->objectName isEqualToString:@"LeftPaddle"]){
                    [parentObj RegisterHit:@"LeftPaddle"];    // assumes RegisterHit is a callback function to register collision
                }
                if([bodyData->objectName isEqualToString:@"RightPaddle"]){
                    [parentObj RegisterHit:@"RightPaddle"];    // assumes RegisterHit is a callback function to register collision
                }
                if([bodyData->objectName isEqualToString:@"LeftWall"]){
                    [parentObj RegisterHit:@"LeftWall"];    // call registerhit to signal that left wall was hit
                }
                if([bodyData->objectName isEqualToString:@"RightWall"]){
                    [parentObj RegisterHit:@"RightWall"];    // call registerhit to signal that right wall was hit
                }
            }
        }
    }
    void PostSolve(b2Contact* contact, const b2ContactImpulse* impulse) {};
};

#pragma mark - CBox2D

@interface CBox2D ()
{
    // Box2D-specific objects
    b2Vec2 *gravity;
    b2World *world;

    b2Body *theLeftWall, *theRightWall, *theGround, *theBall, *theRoof, *theRightPaddle, *theLeftPaddle;
    
    UserData *ballData, *leftWallData, *rightWallData, *rightPaddleData, *leftPaddleData;

    
    CContactListener *contactListener;
    CGFloat width, height;
    float totalElapsedTime;
    // You will also need some extra variables here for the logic
    bool ballHitLeftWall, ballHitRightWall;
    bool ballHitLeftPaddle, ballHitRightPaddle;
    bool ballLaunched, paddleMoved;
    bool scored;
    bool obstacleHitCleaner;
}
@end

@implementation CBox2D

@synthesize xDir, yDir, playerYDir;
@synthesize gameStart;
@synthesize ball;

- (instancetype)init
{
    self = [super init];
    if (self) {
        // Initialize Box2D
        gravity = new b2Vec2(0.0f, GRAVITY);
        world = new b2World(*gravity);
        
        b2BodyDef gdBodyDef;
        gdBodyDef.type = b2_staticBody;
        gdBodyDef.position.Set(GROUND_ROOF_POS_X, GROUND_ROOF_PADDING);//width, height of the ground
        theGround = world->CreateBody(&gdBodyDef);
        b2PolygonShape gdBox;
        gdBox.SetAsBox(GROUND_ROOF_WIDTH, GROUND_ROOF_HEIGHT);
        theGround->CreateFixture(&gdBox, 0.0f);
        
        b2BodyDef rfBodyDef;
        rfBodyDef.type = b2_staticBody;
        rfBodyDef.position.Set(GROUND_ROOF_POS_X, SCREEN_BOUNDS_Y - GROUND_ROOF_PADDING);
        theRoof = world->CreateBody(&rfBodyDef);
        b2PolygonShape rfBox;
        rfBox.SetAsBox(GROUND_ROOF_WIDTH, GROUND_ROOF_HEIGHT);// physical box
        theRoof->CreateFixture(&rfBox, 0.0f);

        // For brick & ball sample
        contactListener = new CContactListener();
        world->SetContactListener(contactListener);
        
        // Set up the brick and ball objects for Box2D
        b2BodyDef leftwallBodyDef;
        leftwallBodyDef.type = b2_staticBody;
        leftwallBodyDef.position.Set(Left_Wall_POS_X, Left_Wall_POS_Y);
        theLeftWall = world->CreateBody(&leftwallBodyDef);
        
        leftWallData = new UserData(self,@"LeftWall");
        
        if (theLeftWall)
        {
            theLeftWall->SetUserData((void *)leftWallData);
            theLeftWall->SetAwake(false);
            b2PolygonShape staticBox;
            staticBox.SetAsBox(Left_Wall_WIDTH/2, Left_Wall_HEIGHT/2);
            b2FixtureDef fixtureDef;
            fixtureDef.shape = &staticBox;
            fixtureDef.density = 1.0f;
            fixtureDef.friction = 0.3f;
            fixtureDef.restitution = 0.0f;
            theLeftWall->CreateFixture(&fixtureDef);
        }
        
        // Set up the brick and ball objects for Box2D
        b2BodyDef rightwallBodyDef;
        rightwallBodyDef.type = b2_staticBody;
        rightwallBodyDef.position.Set(SCREEN_BOUNDS_X, Left_Wall_POS_Y);
        theRightWall = world->CreateBody(&rightwallBodyDef);
        
        rightWallData = new UserData(self,@"RightWall");
        
        if (theRightWall)
        {
            theRightWall->SetUserData((void *)rightWallData);
            theRightWall->SetAwake(false);
            b2PolygonShape staticBox;
            staticBox.SetAsBox(Left_Wall_WIDTH/2, Left_Wall_HEIGHT/2);
            b2FixtureDef fixtureDef;
            fixtureDef.shape = &staticBox;
            fixtureDef.density = 1.0f;
            fixtureDef.friction = 0.3f;
            fixtureDef.restitution = 0.0f;
            theRightWall->CreateFixture(&fixtureDef);
        }
      
        //ball definition
        ball = [[Ball alloc]init];
        b2BodyDef ballBodyDef;
        ballBodyDef.type = b2_dynamicBody;
        ballBodyDef.position.Set(BALL_POS_X, BALL_POS_Y);
        theBall = world->CreateBody(&ballBodyDef);
        
        ballData = new UserData(self, @"Ball");
        
        if (theBall)
        {
            theBall->SetUserData((void *)ballData);
            theBall->SetAwake(false);
            b2CircleShape circle;
            circle.m_p.Set(0, 0);
            circle.m_radius = BALL_RADIUS;
            b2FixtureDef circleFixtureDef;
            circleFixtureDef.shape = &circle;
            circleFixtureDef.density = 0.1f;
            circleFixtureDef.friction = 0.0f;
            circleFixtureDef.restitution = 1.0f;
            theBall->CreateFixture(&circleFixtureDef);
        }

        //paddle definition
        b2BodyDef lPaddleBodyDef;
        lPaddleBodyDef.type = b2_dynamicBody;
        lPaddleBodyDef.position.Set(PADDLE_LEFT_POS_X, PADDLE_POS_Y);
        theLeftPaddle = world->CreateBody(&lPaddleBodyDef);
        
        leftPaddleData = new UserData(self, @"LeftPaddle");
        
        if (theLeftPaddle)
        {
            theLeftPaddle->SetUserData((void *)leftPaddleData);
            theLeftPaddle->SetAwake(false);
            b2PolygonShape dynamicBox;
            dynamicBox.SetAsBox(PADDLE_WIDTH/2, PADDLE_HEIGHT/2);
            b2FixtureDef fixtureDef;
            fixtureDef.shape = &dynamicBox;
            fixtureDef.density = 1.0f;
            fixtureDef.friction = 0.0f;
            fixtureDef.restitution = 0.0f;
            theLeftPaddle->CreateFixture(&fixtureDef);
        }
        
        //obstacle definition
        b2BodyDef rPaddleBodyDef;
        rPaddleBodyDef.type = b2_dynamicBody;
        rPaddleBodyDef.position.Set(PADDLE_RIGHT_POS_X, PADDLE_POS_Y);
        theRightPaddle = world->CreateBody(&rPaddleBodyDef);
        
        rightPaddleData = new UserData(self, @"RightPaddle");
        
        if (theRightPaddle)
        {
            
            theRightPaddle->SetUserData((void *)rightPaddleData);
            theRightPaddle->SetAwake(false);
            b2PolygonShape dynamicBox;
            dynamicBox.SetAsBox(PADDLE_WIDTH/2, PADDLE_HEIGHT/2);
            b2FixtureDef fixtureDef;
            fixtureDef.shape = &dynamicBox;
            fixtureDef.density = 1.0f;
            fixtureDef.friction = 0.0f;
            fixtureDef.restitution = 0.0f;
            theRightPaddle->CreateFixture(&fixtureDef);
        }
        
        totalElapsedTime = 0;
        ballHitLeftWall = false;
        ballLaunched = false;
        scored = false;
        gameStart = false;
    }
    return self;
}

- (void)dealloc
{
    if (gravity) delete gravity;
    if (world) delete world;
    if (contactListener) delete contactListener;
}

-(void)Update:(float)elapsedTime
{
    // Check here if we need to launch the ball
    //  and if so, use ApplyLinearImpulse() and SetActive(true)
    if(theBall) {
        theBall->SetActive(true);
    }
    if(paddleMoved){
        theLeftPaddle->SetActive(true);
        theLeftPaddle->SetLinearVelocity(b2Vec2(0, -playerYDir* 1000));
        paddleMoved = false;
    }
    
    if(theLeftPaddle){
        theLeftPaddle->SetTransform(b2Vec2(PADDLE_LEFT_POS_X, theLeftPaddle->GetPosition().y), theLeftPaddle->GetAngle());
        theLeftPaddle->SetAwake(true);
    }
    
    if(theRightPaddle){
        theRightPaddle->SetTransform(b2Vec2(PADDLE_RIGHT_POS_X,theBall->GetPosition().y), theRightPaddle->GetAngle());
        theRightPaddle->SetAwake(true);
    }
    
    //Makes the ground and roof in the viewport
    if (theGround){
        theGround->SetTransform(b2Vec2(GROUND_ROOF_POS_X,0), theGround->GetAngle());
        theGround->SetAwake(true);
    }
    
    if (theRoof){
        theRoof->SetTransform(b2Vec2(GROUND_ROOF_POS_X,SCREEN_BOUNDS_Y), theRoof->GetAngle());
        theRoof->SetAwake(true);
    }
    
    if(theLeftWall){
        theLeftWall->SetTransform(b2Vec2(0 ,SCREEN_BOUNDS_Y/2), theLeftWall->GetAngle());
    }
    
    if(theRightWall){
        theRightWall->SetTransform(b2Vec2(SCREEN_BOUNDS_X ,SCREEN_BOUNDS_Y/2), theRightWall->GetAngle());
    }
    
    // If the last collision test was positive,
    // increase speed
    if (ballHitLeftPaddle)
    {    printf("Left ");
        theBall->ApplyLinearImpulse(b2Vec2(VELOCITY_INCREASE,0), theBall->GetPosition(), true);
        ballHitLeftPaddle = false;
    } else if (ballHitRightPaddle) {
        printf("Right ");
        theBall->ApplyLinearImpulse(b2Vec2(-VELOCITY_INCREASE,0), theBall->GetPosition(), true);
        ballHitRightPaddle = false;
    }
    
    //Check if the ball hit either walls, if true score of the side increase.
    if(ballHitLeftWall){
        ballHitLeftWall = false;
        [self Reset];
    } else if (ballHitRightWall){
        ballHitRightWall = false;
        [self Reset];
    }
        
    if (world)
    {
        while (elapsedTime >= MAX_TIMESTEP)
        {
            world->Step(MAX_TIMESTEP, NUM_VEL_ITERATIONS, NUM_POS_ITERATIONS);
            elapsedTime -= MAX_TIMESTEP;
        }
        
        if (elapsedTime > 0.0f)
        {
            world->Step(elapsedTime, NUM_VEL_ITERATIONS, NUM_POS_ITERATIONS);
        }
    }
}

//Check the name of the object when the player collides with that object
-(void)RegisterHit:(NSString *) objectName
{
    if([objectName  isEqual: @"LeftPaddle"]){
        ballHitLeftPaddle = true;
    }
    if([objectName  isEqual: @"RightPaddle"]){
        ballHitRightPaddle = true;
    }
    if([objectName  isEqual: @"LeftWall"]){
        ballHitLeftWall = true;
    }
    if([objectName  isEqual: @"RightWall"]){
        ballHitRightWall = true;
    }
}

-(void)LaunchBall
{
    // Set some flag here for processing later...
    theBall->SetLinearVelocity(b2Vec2(300, 300));
    ballLaunched = true;
}

-(void)UpdatePaddle:(float)posY
{
    b2Vec2 paddlePos = theLeftPaddle->GetPosition();
    paddlePos.y = -((paddlePos.y/SCREEN_BOUNDS_Y)-1);
    playerYDir = posY - paddlePos.y;
    paddleMoved = true;
}

//if the ball hits any end goal, the ball and the both paddles should be centered on their initial location.
-(void)Reset
{
    gameStart = false;
    scored = false;
    theBall->SetLinearVelocity(b2Vec2(0,0));
    theBall->SetTransform(b2Vec2(BALL_POS_X,BALL_POS_Y), theBall->GetAngle());
    //theBall->SetAwake(true);
}

-(void *)GetObjectPositions
{
    auto *objPosList = new std::map<const char *,b2Vec2>;
    if (theBall)
        (*objPosList)["ball"] = theBall->GetPosition();
    if (theLeftWall)
        (*objPosList)["leftwall"] = theLeftWall->GetPosition();
    if (theRightWall)
        (*objPosList)["rightwall"] = theRightWall->GetPosition();
    if (theLeftPaddle)
        (*objPosList)["leftpaddle"] = theLeftPaddle->GetPosition();
    if (theRightPaddle)
        (*objPosList)["rightpaddle"] = theRightPaddle->GetPosition();
    if (theGround)
        (*objPosList)["ground"] = theGround->GetPosition();
    if (theRoof)
        (*objPosList)["roof"] = theRoof->GetPosition();
    return reinterpret_cast<void *>(objPosList);
}

@end
