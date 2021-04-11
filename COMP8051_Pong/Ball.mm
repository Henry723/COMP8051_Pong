//
//  Ball.mm
//  COMP8051_Pong
//
//  Created by Henry Zhang on 2021-04-11.
//

#include "Ball.h"

@implementation Ball

@synthesize posX, posY, initialJump;

-(instancetype)init {
    self = [super init];
    if(self){
    }
    return self;
}

-(void) updatePos:(float)positionX :(float)positionY{
    posX = positionX;
    posY = positionY;
}

-(void) dealloc{
    
}

-(void) checkCollision:(float)positionX:(float)positionY:(float)width:(float)height{
    
    if(posX >= positionX - width/2 &&
        posX <= positionX + width/2){

        if(posY > positionY + height/2){
            printf("Top \n");
        }
        else if(posY < positionY - height/2){
                    //change the enum for which side its colliding with to the enum
            printf("Bottom \n");
        }
    }
    else {
        if(posX < positionX + width/2){
            printf("Left \n");
        } else if(posX > positionX - width/2){
            printf("Right \n");
        }
    }
}

@end

