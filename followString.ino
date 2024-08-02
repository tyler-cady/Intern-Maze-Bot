void followString(string path){
    for (int i = 0; i < path.length(); i++){
        if (path[i] == 'F'){ 
            forward_1_block();
            
        }
        else if (path[i] == 'R') {
            turn_90(true);//right 
            forward_1_block();
        }
        
        else if (path[i] == 'L'){
            turn_90(false);//left
            API::moveForward(1);
            
        }
        else if (path[i] == 'B') {
            turn_90(true);
            turn_90(true)
            forward_1_block();
        }
    }
}

int main(){
    string path = "RFFLRLFRLFFLFFFFLFFFRLLFFLRLFFLFL";
    followString(path);

}
