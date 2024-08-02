void followString(string path){
    for (int i = 0; i < path.length(); i++){
        if (path[i] == 'F'){ 
            forward_1_block();
            
        }
        else if (path[i] == 'R') {
            turn(true);//right 
            forward_1_block();
        }
        
        else if (path[i] == 'L'){
            turn(false);//left
            API::moveForward(1);
            
        }
        else if (path[i] == 'B') {
            turn(true);
            turn(true)
            forward_1_block();
        }
    }
}

int main(){
    string path = "RFFLRLFRLFFLFFFFLFFFRLLFFLRLFFLFL";
    followString(path);

}
