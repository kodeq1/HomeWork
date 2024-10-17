import java.util.Random;
import java.util.Scanner;


public class Main {


    public static Double RandomInt(Double min, Double max) {
        Random rnd = new Random();
        return rnd.nextDouble(max - min);

    }
    public static void Main(String[] args) {
        Double totalCash = 10000.0;
        Scanner sc = new Scanner(System.in);
        System.out.println("hesabınızdaki para : " + totalCash);
        System.out.println("Yatırmak İstediğiniz Parayı Giriniz: ");


        while(totalCash >0 && totalCash < 200000) {
            int playerInput = sc.nextInt();
            if (playerInput > totalCash) {
                System.out.println("paranız yeterli değil");
                System.out.println("hesabınızdaki para : " + totalCash);
                continue;
            }
            Double gamble = Double.valueOf(playerInput);
            totalCash = totalCash - gamble;

            Double total = 0.0;
            for (int i=0 ; i<=5 ;i++) {
                Double randomNum = RandomInt(0.1,1.5);
                gamble = randomNum   * gamble;
                total = total + gamble;
            }
            totalCash = totalCash + total;
            System.out.println(totalCash);
        }
        System.out.println("tebrikler kazandınız");
    }

}
