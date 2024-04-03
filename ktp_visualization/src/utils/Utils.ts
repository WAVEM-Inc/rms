export const KTP_DEV_ID: string = "KECDSEMITB001";

export function getCurrentTime(): string {
    const now: Date = new Date();
    const year: string = now.getFullYear().toString().slice(-2);
    const month: string = ('0' + (now.getMonth() + 1)).slice(-2);
    const date: string = ('0' + now.getDate()).slice(-2);
    const hours: string = ('0' + now.getHours()).slice(-2);
    const minutes: string = ('0' + now.getMinutes()).slice(-2);
    const seconds: string = ('0' + now.getSeconds()).slice(-2);
    const milliseconds: string = ('00' + now.getMilliseconds()).slice(-3);

    const currentTime: string = year + month + date + hours + minutes + seconds + milliseconds;
    return currentTime;
}