//
//  Ball.h
//  COMP8051_Pong
//
//  Created by Henry Zhang on 2021-04-11.
//

#ifndef Ball_h
#define Ball_h
#import <Foundation/Foundation.h>
#include <time.h>
#include <stdlib.h>

@interface Ball : NSObject{}
@property (nonatomic, readwrite) float posX, posY;
@property (nonatomic, readwrite)bool scored, initialJump;

-(void)updatePos:(float)positionX:(float)positionY;
-(void)dealloc;
@end

#endif /* Ball_h */
