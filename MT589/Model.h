#ifndef MODEL_H
#define MODEL_H

#include <vector>
#include <string>
#include <bitset>

// Address in matrix
struct Point {
    Point() {
        this->row = -1;
        this->col = -1;
    }

    Point(int row, int col) {
        this->row = row;
        this->col = col;
    }

    int row;

    int col;

    bool isNull() {
        bool isNull = (row == -1) && (col == -1);
        return isNull;
    }

    static Point nullPoint() {
        return Point();
    }

    friend bool operator== (Point &lhs, Point &rhs) {
        return (lhs.row == rhs.row) && (lhs.col == rhs.col);
    }

    friend bool operator!= (Point &lhs, Point &rhs) {
        return (lhs.row != rhs.row) || (lhs.col != rhs.col);
    }
};

// Mode
enum Mode {
    editing,
    running
};

// UI Model
class Model {

    Mode mode = editing;

public:
    Model() { }

    Mode getMode() {
        return mode;
    }

    void setMode(Mode mode) {
        this->currentPoint = Point::nullPoint();
        this->mode = mode;
    }

    Point currentPoint = Point::nullPoint();

    Point startPoint = Point::nullPoint();

    std::string current_filename = "";

    std::vector<std::string> mnemonics = {
        "ILR",
        "ACM",
        "SRA",
        "ALR",
        "AMA",
        "LMI",
        "LMM",
        "CIA",
        "DSM",
        "LDM",
        "DCA",
        "CSR",
        "CSA",
        "SDR",
        "SDA",
        "LDI",
        "INR",
        "INA",
        "ADR",
        "AIA",
        "CLR",
        "CLA",
        "ANR",
        "ANM",
        "ANI",
        "TZR",
        "LTM",
        "TZA",
        "NOP",
        "LMF",
        "ORR",
        "ORM",
        "ORI",
        "CMR",
        "LCM",
        "CMA",
        "XNR",
        "XNM",
        "XNI",
        "CONNECT EXTERNAL" // общение с внешним
      };

    std::vector<std::string> funcs = {
        "0001001",
        "0001010",
        "0001110",
        "0001001",
        "0001010",

        "0011001",
        "0011010",
        "0011110",
        "0011001",
        "0011010",
        "0011110",

        "0101001",
        "0101010",
        "0101001",
        "0101010",
        "0101110",

        "0111001",
        "0111110",
        "0111001",
        "0111110",

        "1001001",
        "1001010",
        "1001001",
        "1001010",
        "1001110",

        "1011001",
        "1011010",
        "1011110",

        "1101001",
        "1101010",
        "1101001",
        "1101010",
        "1101110",

        "1111001",
        "1111010",
        "1111110",
        "1111001",
        "1111010",
        "1111110",
        "1111111"  // общение с внешним устр-вом
      };

    std::vector<std::string> ks = {
        "00",
        "00",
        "00",
        "11",
        "11",
        "00",
        "00",
        "00",
        "11",
        "11",
        "11",
        "00",
        "00",
        "11",
        "11",
        "11",
        "00",
        "00",
        "11",
        "11",
        "00",
        "00",
        "11",
        "11",
        "11",
        "11",
        "11",
        "11",
        "00",
        "00",
        "11",
        "11",
        "11",
        "00",
        "00",
        "00",
        "11",
        "11",
        "11",
        "00" // для CONNECT EXTERNAL
      };

    std::vector<std::vector<std::string>> regsMnemonics = {
              {
                  "RO",
                  "R1",
                  "R2",
                  "R3",
                  "R4",
                  "R5",
                  "R6",
                  "R7",
                  "R8",
                  "R9",
                  "T",
                  "AC"
              },
              {
                  "T",
                  "AC"
              },
              {
                  "T",
                  "AC"
              }
      };

    std::vector<std::vector<std::string>> regAddresses = {
            {
              "0000",
              "0001",
              "0010",
              "0011",
              "0100",
              "0101",
              "0110",
              "0111",
              "1000",
              "1001",
              "1100",
              "1101"
          },
          {
              "1010",
              "1011"
          },
          {
              "1110",
              "1111"
          }
      };

    std::vector<int> r_groups = {
        1,
        2,
        3,
        1,
        2,
        1,
        2,
        3,
        1,
        2,
        3,
        1,
        2,
        1,
        2,
        3,
        1,
        3,
        1,
        3,
        1,
        2,
        1,
        2,
        3,
        1,
        2,
        3,
        1,
        2,
        1,
        2,
        3,
        1,
        2,
        3,
        1,
        2,
        3
      };
};

#endif // MODEL_H
