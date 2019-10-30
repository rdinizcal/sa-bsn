from src import Analyzer as stat
import sys

def main():
    analyzer = stat.Analyzer(len(sys.argv), sys.argv)
    analyzer.run()

if __name__ == "__main__":
    main()