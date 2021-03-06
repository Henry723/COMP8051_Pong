//
//  Copyright © Borna Noureddin. All rights reserved.
//

#ifndef MyGLGame_CBox2D_h
#define MyGLGame_CBox2D_h

#import <UIKit/UIKit.h>
#import <GLKit/GLKit.h>

#import "Ball.h"

// Set up brick and ball physics parameters here:
//   position, width+height (or radius), velocity,
//   and how long to wait before dropping brick
#define SCREEN_BOUNDS_X        800
#define SCREEN_BOUNDS_Y        600

// Physics & Game-Speed Parameters
#define GRAVITY                0
#define REFRESH_RATE           0.05/60


#define Left_Wall_POS_X            0
#define Left_Wall_POS_Y            300
#define Left_Wall_WIDTH            50.0f
#define Left_Wall_HEIGHT           600.0f

#define BALL_POS_X             SCREEN_BOUNDS_X/2
#define BALL_POS_Y             SCREEN_BOUNDS_Y/2
#define BALL_RADIUS            20.0f
#define BALL_VELOCITY          400.0f
#define BALL_SPHERE_SEGS       128
#define VELOCITY_INCREASE      10000.0f
#define GROUND_ROOF_PADDING    10.0f
#define GROUND_ROOF_POS_X      400
#define GROUND_ROOF_WIDTH      800.0f
#define GROUND_ROOF_HEIGHT     10.0f

#define PADDLE_LEFT_POS_X     100
#define PADDLE_RIGHT_POS_X     SCREEN_BOUNDS_X - 100
#define PADDLE_POS_Y         300
#define PADDLE_WIDTH         40.0f
#define PADDLE_HEIGHT        100.0f

#define GAME_SPEED             5

@interface CBox2D : NSObject

@property int playerScore,aiScore;
@property float yDir;
@property float playerYDir;
@property bool gameStart;
@property (nonatomic) Ball * ball;

-(void) Update:(float)elapsedTime;  // update the Box2D engine
-(void) RegisterHit:(NSString *) objectName;// Register when the ball hits the brick
-(void *)GetObjectPositions;        // Get the positions of the ball and brick

-(void) LaunchBall;
-(void) UpdatePaddle:(float)posY;
-(void) Reset;

@end

#endif
