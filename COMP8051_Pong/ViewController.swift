//
//  Copyright © Borna Noureddin. All rights reserved.
//

import GLKit

extension ViewController: GLKViewControllerDelegate {
    func glkViewControllerUpdate(_ controller: GLKViewController) {
        glesRenderer.update()
    }
}

class ViewController: GLKViewController {
    
    private var context: EAGLContext?
    private var glesRenderer: Renderer!
        
    private func setupGL() {
        context = EAGLContext(api: .openGLES3)
        EAGLContext.setCurrent(context)
        if let view = self.view as? GLKView, let context = context {
            view.context = context
            delegate = self as GLKViewControllerDelegate
            glesRenderer = Renderer()
            glesRenderer.setup(view)
            glesRenderer.loadModels()
        }
    }
    
    override func viewDidLoad() {
        super.viewDidLoad()
        setupGL()
        
    }
    
        
    @IBAction func TapAndHold(_ sender: UILongPressGestureRecognizer) {
        // Disable input if the GameDirector
    //        float xPos;
        let tapLocation = sender.location(in: sender.view)
            
        let screenSize: CGRect = UIScreen.main.bounds;
        let xPos : Float = Float(tapLocation.x / screenSize.width);
        let yPos : Float = Float(tapLocation.y / screenSize.height);
            
            
        if sender.state == .began {
            if(!glesRenderer.box2d.gameStart){
                glesRenderer.box2d.gameStart = true;
                glesRenderer.box2d.launchBall();
            }
            glesRenderer.box2d.updatePaddle(yPos);
        } else if sender.state == .changed {
            glesRenderer.box2d.updatePaddle(yPos);
        } else if sender.state == .ended {}
    }
    
    override func glkView(_ view: GLKView, drawIn rect: CGRect) {
        glesRenderer.draw(rect)
    }

}
