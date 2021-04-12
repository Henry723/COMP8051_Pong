//
//  Ball.mm
//  COMP8051_Pong
//
//  Created by Henry Zhang on 2021-04-11.
//

#include "Ball.h"

@implementation Ball

@synthesize posX, posY;

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

@end

