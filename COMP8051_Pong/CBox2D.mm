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

    b2Body *theLeftWall, *theRightWall, *theGround, *theBall, *theRoof, *theRightPaddle, *theleftPaddle;
    
    UserData *ballData, *wallData, *rightPaddleData, *groundData, *leftPaddleData;

    
    CContactListener *contactListener;
    CGFloat width, height;
    float totalElapsedTime;
    // You will also need some extra variables here for the logic
    bool ballHitLeftWall, ballHitRightWall;
    bool ballHitLeftPaddle, ballHitRightPaddle;
    bool ballLaunched;
    bool obstacleHitCleaner;
}
@end

@implementation CBox2D

@synthesize xDir, yDir;
@synthesize scored;
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
        
        //ground counts as obstacle, since obstacles are non-harmful objects which the ground can be a part of
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
        leftwallBodyDef.type = b2_kinematicBody;
        leftwallBodyDef.position.Set(Left_Wall_POS_X, Left_Wall_POS_Y);
        theLeftWall = world->CreateBody(&leftwallBodyDef);
        
        wallData = new UserData(self,@"LeftWall");
        
        if (theLeftWall)
        {
            theLeftWall->SetUserData((void *)wallData);
            theLeftWall->SetAwake(false);
            b2PolygonShape dynamicBox;
            dynamicBox.SetAsBox(Left_Wall_WIDTH/2, Left_Wall_HEIGHT/2);
            b2FixtureDef fixtureDef;
            fixtureDef.shape = &dynamicBox;
            fixtureDef.density = 1.0f;
            fixtureDef.friction = 0.3f;
            fixtureDef.restitution = 0.0f;
            theLeftWall->CreateFixture(&fixtureDef);
        }
        
        // Set up the brick and ball objects for Box2D
        b2BodyDef rightwallBodyDef;
        rightwallBodyDef.type = b2_kinematicBody;
        rightwallBodyDef.position.Set(Left_Wall_POS_X+800, Left_Wall_POS_Y);
        theRightWall = world->CreateBody(&rightwallBodyDef);
        
        wallData = new UserData(self,@"RightWall");
        
        if (theRightWall)
        {
            theRightWall->SetUserData((void *)wallData);
            theRightWall->SetAwake(false);
            b2PolygonShape dynamicBox;
            dynamicBox.SetAsBox(Left_Wall_WIDTH/2, Left_Wall_HEIGHT/2);
            b2FixtureDef fixtureDef;
            fixtureDef.shape = &dynamicBox;
            fixtureDef.density = 1.0f;
            fixtureDef.friction = 0.3f;
            fixtureDef.restitution = 0.0f;
            theRightWall->CreateFixture(&fixtureDef);
        }
      
        //ball definition
        ball = [[Ball alloc]init];
        ball.initialJump = false;
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

        //obstacle definition
        b2BodyDef lPaddleBodyDef;
        lPaddleBodyDef.type = b2_staticBody;
        lPaddleBodyDef.position.Set(OBSTACLE_POS_X-800, OBSTACLE_POS_Y);
        theleftPaddle = world->CreateBody(&lPaddleBodyDef);
        
        leftPaddleData = new UserData(self, @"LeftPaddle");
        
        if (theleftPaddle)
        {
            
            theleftPaddle->SetUserData((void *)leftPaddleData);
            theleftPaddle->SetAwake(false);
            b2PolygonShape staticBox;
            staticBox.SetAsBox(OBSTACLE_WIDTH/2, OBSTACLE_HEIGHT/2);
            b2FixtureDef fixtureDef;
            fixtureDef.shape = &staticBox;
            fixtureDef.density = 1.0f;
            fixtureDef.friction = 0.3f;
            fixtureDef.restitution = 0.0f;
            theleftPaddle->CreateFixture(&fixtureDef);
        }
        
        //obstacle definition
        b2BodyDef rPaddleBodyDef;
        rPaddleBodyDef.type = b2_staticBody;
        rPaddleBodyDef.position.Set(OBSTACLE_POS_X-200, OBSTACLE_POS_Y);
        theRightPaddle = world->CreateBody(&rPaddleBodyDef);
        
        rightPaddleData = new UserData(self, @"RightPaddle");
        
        if (theRightPaddle)
        {
            
            theRightPaddle->SetUserData((void *)rightPaddleData);
            theRightPaddle->SetAwake(false);
            b2PolygonShape staticBox;
            staticBox.SetAsBox(OBSTACLE_WIDTH/2, OBSTACLE_HEIGHT/2);
            b2FixtureDef fixtureDef;
            fixtureDef.shape = &staticBox;
            fixtureDef.density = 1.0f;
            fixtureDef.friction = 0.3f;
            fixtureDef.restitution = 0.0f;
            theRightPaddle->CreateFixture(&fixtureDef);
        }
        
        totalElapsedTime = 0;
        ballHitLeftWall = false;
        ballLaunched = false;
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
    if (ballLaunched)
    {
        theBall->SetLinearVelocity(b2Vec2(xDir * BALL_VELOCITY, yDir * BALL_VELOCITY));
        theBall->SetActive(true);
        
        ballLaunched = false;
    }
    
    //Check if the ball hit either walls, if true score of the side increase.
    if(ballHitLeftWall){
        ballHitLeftWall = false;
        scored = true;
    } else if (ballHitRightWall){
        ballHitRightWall = false;
        scored = true;
    }
    
    //in case the ball is not scored on either end, therefore dont update playerposition
    if(!scored){
        [ball updatePos:theBall->GetPosition().x :theBall->GetPosition().y];
    } else {
        [self Reset];
    }
    
    // If the last collision test was positive,
    // increase speed
    if (ballHitLeftPaddle)
    {    printf("Left ");
        theBall->ApplyLinearImpulse(b2Vec2(VELOCITY_INCREASE,0), b2Vec2(theBall->GetPosition()), true);
        ballHitLeftPaddle = false;
    } else if (ballHitRightPaddle) {
        printf("Right ");
        theBall->ApplyLinearImpulse(b2Vec2(-VELOCITY_INCREASE,0), b2Vec2(theBall->GetPosition()), true);
        ballHitRightPaddle = false;
    }
    
    if(theleftPaddle)
        theleftPaddle->SetAwake(true);

    if(theRightPaddle)
        theRightPaddle->SetAwake(true);
    
    //Makes the ground and roof in sync of viewport
    if (theGround){
        theGround->SetTransform(b2Vec2(GROUND_ROOF_POS_X,0), theGround->GetAngle());
        theGround->SetAwake(true);
    }
    
    if (theRoof){
        theRoof->SetTransform(b2Vec2(GROUND_ROOF_POS_X,SCREEN_BOUNDS_Y), theGround->GetAngle());
        theRoof->SetAwake(true);
    }
    
    if(theLeftWall){
        theLeftWall->SetTransform(b2Vec2(0 ,SCREEN_BOUNDS_Y/2), theLeftWall->GetAngle());
    }
    
    if(theRightWall){
        theRightWall->SetTransform(b2Vec2(SCREEN_BOUNDS_X ,SCREEN_BOUNDS_Y/2), theRightWall->GetAngle());
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

-(void) SetTargetVector:(float)posX :(float)posY
{
    // Curate ball Pos value to be scaled to screen space.
    b2Vec2 currentBallPos = theBall->GetPosition();
    currentBallPos.x = ((currentBallPos.x)/ SCREEN_BOUNDS_X);
    
    currentBallPos.y = -((currentBallPos.y / SCREEN_BOUNDS_Y) - 1);
    
    // Direction will be the vector between the two curated points.
    xDir = posX - currentBallPos.x;
    yDir =  currentBallPos.y - posY;
    
}

// Halt current velocity, set initial target position
-(void)InitiateNewJump:(float)posX :(float)posY
{
    // Curate ball Pos value to be scaled to screen space.
    b2Vec2 currentBallPos = theBall->GetPosition();
    currentBallPos.x = ((currentBallPos.x) / SCREEN_BOUNDS_X);
    
    currentBallPos.y = -((currentBallPos.y / SCREEN_BOUNDS_Y) - 1);
    
    // Direction will be the vector between the two curated points.
    xDir = posX - currentBallPos.x;
    yDir =  currentBallPos.y - posY;
    
    // Normalize the values
    float vectorMagnitude = sqrt((pow(xDir, 2) + pow(yDir, 2)));
    xDir = xDir / vectorMagnitude;
    yDir = yDir / vectorMagnitude;
    
    printf("New tap, velocity targer %4.2f, %4.2f...\n", xDir, yDir);
}

// Update current position vector
-(void)UpdateJumpTarget:(float)posX :(float)posY
{
    
    // Curate ball Pos value to be scaled to screen space.
    b2Vec2 currentBallPos = theBall->GetPosition();
    currentBallPos.x = ((currentBallPos.x)/ SCREEN_BOUNDS_X);
    
    currentBallPos.y = -((currentBallPos.y / SCREEN_BOUNDS_Y) - 1);
    
    // Direction will be the vector between the two curated points.
    
    xDir = posX - currentBallPos.x;
    yDir = currentBallPos.y - posY;
    
    // Normalize the values
    float vectorMagnitude = sqrt((pow(xDir, 2) + pow(yDir, 2)));
    xDir = xDir / vectorMagnitude;
    yDir = yDir / vectorMagnitude;
    
    printf("Updating Velocity Target to %4.2f, %4.2f..\n", xDir, yDir);
}

//
-(void)LaunchJump
{
    // Set some flag here for processing later...
    ballLaunched = true;
}

//if the ball hits any end goal, the ball and the both paddles should be centered on their initial location.
-(void)Reset
{
    scored = false;
    theBall->SetLinearVelocity(b2Vec2(0,0));
    theBall->SetTransform(b2Vec2(BALL_POS_X,BALL_POS_Y), theBall->GetAngle());
    theBall->SetAwake(true);
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
    if (theleftPaddle)
        (*objPosList)["leftpaddle"] = theleftPaddle->GetPosition();
    if (theRightPaddle)
        (*objPosList)["rightpaddle"] = theRightPaddle->GetPosition();
    if (theGround)
        (*objPosList)["ground"] = theGround->GetPosition();
    if (theRoof)
        (*objPosList)["roof"] = theRoof->GetPosition();
    return reinterpret_cast<void *>(objPosList);
}

@end
