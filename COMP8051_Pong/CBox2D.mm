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
                if([bodyData->objectName isEqualToString:@"Obstacle"]){
                    [parentObj RegisterHit:@"Obstacle"];    // assumes RegisterHit is a callback function to register collision
                }
                if([bodyData->objectName isEqualToString:@"LeftWall"]){
                    [parentObj RegisterHit:@"LeftWall"];    // call registerhit to signal that left wall was hit
                }
                if([bodyData->objectName isEqualToString:@"Ground"]){
                    [parentObj RegisterHit:@"Ground"];    // call registerhit to signal that left wall was hit
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

    b2Body *theLeftWall, *theGround, *theBall, *theRoof, *theObstacle;
    
    UserData *ballData, *wallData, *obstacleData, *groundData;
    
    CContactListener *contactListener;
    CGFloat width, height;
    float totalElapsedTime;
    // You will also need some extra variables here for the logic
    bool ballHitLeftWall;
    bool ballHitObstacle;
    bool ballLaunched;
    bool obstacleHitCleaner;
}
@end

@implementation CBox2D

@synthesize xDir, yDir;
@synthesize dead;
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
        groundData = new UserData(self,@"Ground");
        theGround->SetUserData((void*) groundData);
        

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
        b2BodyDef obstacleBodyDef;
        obstacleBodyDef.type = b2_staticBody;
        obstacleBodyDef.position.Set(OBSTACLE_POS_X-200, OBSTACLE_POS_Y);
        theObstacle = world->CreateBody(&obstacleBodyDef);
        
        obstacleData = new UserData(self, @"Obstacle");
        
        if (theObstacle)
        {
            
            theObstacle->SetUserData((void *)obstacleData);
            theObstacle->SetAwake(false);
            b2PolygonShape staticBox;
            staticBox.SetAsBox(OBSTACLE_WIDTH/2, OBSTACLE_HEIGHT/2);
            b2FixtureDef fixtureDef;
            fixtureDef.shape = &staticBox;
            fixtureDef.density = 1.0f;
            fixtureDef.friction = 0.3f;
            fixtureDef.restitution = 0.0f;
            theObstacle->CreateFixture(&fixtureDef);
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
        theBall->ApplyLinearImpulse(b2Vec2(0, BALL_VELOCITY), theBall->GetPosition(), true);
        theBall->SetLinearVelocity(b2Vec2(xDir * JUMP_MAGNITUDE, yDir * JUMP_MAGNITUDE));
        theBall->SetActive(true);
        
#ifdef LOG_TO_CONSOLE
        NSLog(@"Applying impulse %f to ball\n", BALL_VELOCITY);
#endif
        ballLaunched = false;
    }

    //in case the player is already dead, therefore dont update playerposition
    if(!dead){
        [ball updatePos:theBall->GetPosition().x :theBall->GetPosition().y];
    }
    // Check if it is time yet to drop the brick, and if so
    //  call SetAwake()
    totalElapsedTime += elapsedTime;
    if ((totalElapsedTime > BRICK_WAIT) && theLeftWall)
        theLeftWall->SetAwake(true);
    
    if(ballHitLeftWall){
        world->DestroyBody(theBall);
        theBall = NULL;
        ballHitLeftWall = false;
        dead = true;
    }
    
    // If the last collision test was positive,
    //  stop the ball and destroy the brick
    if (ballHitObstacle)
    {
        ballHitObstacle = false;
    }
    
    if(theObstacle)
        theObstacle->SetAwake(true);

    //Makes the ground and roof in sync of viewport
    if (theGround){
        theGround->SetTransform(b2Vec2(400,0), theGround->GetAngle());
        theGround->SetAwake(true);
    }
    
    if (theRoof){
        theRoof->SetTransform(b2Vec2(400,SCREEN_BOUNDS_Y), theGround->GetAngle());
        theRoof->SetAwake(true);
    }
    
    if(theLeftWall){
        theLeftWall->SetTransform(b2Vec2(0 ,SCREEN_BOUNDS_Y/2), theLeftWall->GetAngle());
    }
    
    if((int)theGround->GetPosition().x - SCREEN_BOUNDS_X/2 >= theObstacle->GetPosition().x) {
        printf("Obsacle in middle of screen\n");
        
        b2BodyDef obstacleBodyDef;
        obstacleBodyDef.type = b2_staticBody;
        obstacleBodyDef.position.Set(theGround->GetPosition().x + SCREEN_BOUNDS_X/2, 200);
        theObstacle = world->CreateBody(&obstacleBodyDef);
        
        UserData* obstacleData = new UserData(self,@"Obstacle");
//        obstacleData->box2D = self;
//        obstacleData->objectName = @"Obstacle";
        if (theObstacle)
        {
            theObstacle->SetUserData((void*) obstacleData);
            theObstacle->SetAwake(false);
            b2PolygonShape staticBox;
            staticBox.SetAsBox(OBSTACLE_WIDTH/2, OBSTACLE_HEIGHT/2);
            b2FixtureDef fixtureDef;
            fixtureDef.shape = &staticBox;
            fixtureDef.density = 1.0f;
            fixtureDef.friction = 0.3f;
            fixtureDef.restitution = 0.0f;
            theObstacle->CreateFixture(&fixtureDef);
        }
        
        
        //theObstacle->SetTransform(b2Vec2(theObstacle->GetPosition().x + OBSTACLE_DISTANCE, obstacle.posY), theObstacle->GetAngle());
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
    if([objectName  isEqual: @"Obstacle"]){
        ballHitObstacle = true;
    }
    if([objectName  isEqual: @"LeftWall"]){
        ballHitLeftWall = true;
    }
    if([objectName  isEqual: @"Ground"]){
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

-(void *)GetObjectPositions
{
    auto *objPosList = new std::map<const char *,b2Vec2>;
    if (theBall)
        (*objPosList)["ball"] = theBall->GetPosition();
    if (theLeftWall)
        (*objPosList)["leftwall"] = theLeftWall->GetPosition();
    if (theObstacle)
        (*objPosList)["obstacle"] = theObstacle->GetPosition();
    if (theGround)
        (*objPosList)["ground"] = theGround->GetPosition();
    if (theRoof)
        (*objPosList)["roof"] = theRoof->GetPosition();
    return reinterpret_cast<void *>(objPosList);
}

@end
