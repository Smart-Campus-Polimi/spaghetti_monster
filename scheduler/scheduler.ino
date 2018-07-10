typedef struct t  {
    unsigned long tStart;
    unsigned long tTimeout;
};

//Tasks and their Schedules.
t t_func1 = {0, 10000}; //Run every 100ms
t t_func2 = {0, 200000}; //Run every 2 seconds.

bool tCheck (struct t *t ) {
  if (millis() > t->tStart + t->tTimeout) {
    return true;  
  }
  else {
    return false;
    }  
}

void tRun (struct t *t) {
    t->tStart = millis();
}

void setup (void) {
  //Arduino setup.
}

void loop (void) {
    if (tCheck(&t_func1)) {
      func1();
      tRun(&t_func1);
    }
    
    if (tCheck(&t_func2)) {
      func2();
      tRun(&t_func2);
    }
}

void func1 (void) {
  Serial.println("asd");
  
  }
void func2 (void) {
  Serial.println("---------");
  //This executes every 2 seconds.
}
