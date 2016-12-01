void computeCoG(double p[4][2], double (&returnArray)[2]){

    int cog_x, cog_y, temp1, temp2;

    for (int var = 0; var < 3; ++var) {
        temp1 = temp1 + p[var][0];
        temp2 = temp2 + p[var][1];
    }

    cog_x = temp1/4;
    cog_y = temp2/4;

    returnArray[0] = cog_x;
    returnArray[1] = cog_y;
}
