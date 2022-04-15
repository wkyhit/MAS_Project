package tileworld.agent;

public class MyMessage extends Message{

    private Object detail[]; //message detail
    public MyMessage(String from, String to, String message, Object[] detail) {
        super(from, to, message);
        this.detail = detail;
    }

    public Object[] getDetail() {
        return detail;
    }
}
